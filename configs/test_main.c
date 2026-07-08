msg : #include "User_Header.h"

#define LED_ON() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET)
#define LED_OFF() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET)

#define PSC 72
#define ARR 100

      const char data_transmit[] = "Shall I compare thee to a summer's day?\n\
	Thou art more lovely and more temperate:\n\
  Rough winds do shake the darling buds of May.";
// const char data_transmit[] = "Shall I compare thee to a summer's day?\n\
// Thou art more lovely and more temperate:\n\
// Rough winds do shake the darling buds of May,\n\
// And summer's lease hath all too short a date:\n\
// Sometime too hot the eye of heaven shines,\n\
// And often is his gold complexion dimm'd;\n\
// And every fair from fair sometime declines,\n\
// By chance or nature's changing course untrimm'd;\n\
// But thy eternal summer shall not fade\n\
// Nor lose possession of that fair thou owest;\n\
// Nor shall Death brag thou wander'st in his shade,\n\
// When in eternal lines to time thou growest:\n\
// So long as men can breathe or eyes can see,\n\
// So long lives this and this gives life to thee.";
// const char data_transmit[] = "S";

// Data encoding config
int data_idx[11] = {0, 1, 2, 3, 4, 5, 6, 8, 9, 10, 12};
int prev_bit = 0;

// #define LONG_START_LEN 198
#define LONG_START_LEN 20
#define SHORT_START_LEN 6
int startseq[SHORT_START_LEN] = {0, 1, 0, 0, 0, 1};

// Ring buffer
#define LED_QUEUE_LEN 2048
uint8_t led_queue[LED_QUEUE_LEN];
// prepare the data - write to led_queue
volatile int write_idx = 0;
// read data from led_queue, blink LED
volatile int read_idx = 0;

// Long start mode
volatile int long_start_mode = 0;
volatile int long_start_counter = 0;

// Message position
int next_char = 0;

const int msg_type_len = 2;
const int msg_data_len = 24;
const int msg_bits_len = 32;
const int led_bits_len = msg_bits_len * 2;
static int packet_count = 0;

extern TIM_HandleTypeDef htim1;
void SystemClock_Config(void);

// --- Hamming + Manchester encoding ---

void encode_hamming_16_11(int msg_type[3], int msg_data[8], int msg_bits[16]) {
  int msg[11], i, total = 0;
  for (i = 0; i < 3; i++)
    msg[i] = msg_type[i];
  for (i = 0; i < 8; i++)
    msg[3 + i] = msg_data[i];

  msg_bits[7] =
      (msg[0] + msg[1] + msg[2] + msg[3] + msg[4] + msg[5] + msg[6]) % 2;
  msg_bits[11] =
      (msg[0] + msg[1] + msg[2] + msg[3] + msg[7] + msg[8] + msg[9]) % 2;
  msg_bits[13] =
      (msg[0] + msg[1] + msg[4] + msg[5] + msg[7] + msg[8] + msg[10]) % 2;
  msg_bits[14] =
      (msg[0] + msg[2] + msg[4] + msg[6] + msg[7] + msg[9] + msg[10]) % 2;

  for (i = 0; i < 11; i++)
    msg_bits[data_idx[i]] = msg[i];
  for (i = 0; i < 15; i++)
    total += msg_bits[i];
  msg_bits[15] = total % 2;
}

void encode_hamming_32_26(int msg_type[2], int msg_data[24], int msg_bits[32]) {
  int msg[26], i, j, data_pos, total = 0;

  // Merge msg_type and msg_data into msg[26]
  msg[0] = msg_type[0];
  msg[1] = msg_type[1];
  for (i = 0; i < 24; i++)
    msg[2 + i] = msg_data[i];

  // Place message bits into msg_bits (skip parity bit positions: 1,2,4,8,16,32)

  for (i = 1; i <= 32; i++) {
    if (i == 1 || i == 2 || i == 4 || i == 8 || i == 16 || i == 32)
      continue;
    msg_bits[i - 1] = msg[data_pos++];
  }

  // Compute Hamming parity bits (at positions: 1, 2, 4, 8, 16)
  for (i = 0; i < 5; i++) {
    int parity_pos = 1 << i; // 1, 2, 4, 8, 16
    int parity = 0;
    for (j = 1; j <= 31; j++) {
      if (j & parity_pos && j != parity_pos) {
        parity ^= msg_bits[j - 1];
      }
    }
    msg_bits[parity_pos - 1] = parity;
  }

  // Compute overall parity at position 32 (index 31)
  for (i = 0; i < 31; i++) {
    total += msg_bits[i];
  }
  msg_bits[31] = total % 2;
}

void convert_to_led_bits(int msg_bits[msg_bits_len],
                         int led_bits[led_bits_len]) {
  int i;
  for (i = 0; i < msg_bits_len; i++) {
    int base = i * 2;
    if (msg_bits[i]) {
      led_bits[base + 0] = 0;
      led_bits[base + 1] = 1;
      prev_bit = 1;
    } else {
      if (prev_bit) {
        led_bits[base + 0] = 0;
        led_bits[base + 1] = 0;
      } else {
        led_bits[base + 0] = 1;
        led_bits[base + 1] = 0;
      }
      prev_bit = 0;
    }
  }
  prev_bit = 0;
}

// --- Queue logic ---

#define NEXT_IDX(idx) (((idx) + 1) % LED_QUEUE_LEN)

void enqueue_char_led_bits(char c) {
  int i;
  int msg_type[msg_type_len] = {0, 0};
  int msg_data[msg_data_len], msg_bits[msg_bits_len], led_bits[led_bits_len];

  int space = (read_idx > write_idx)
                  ? (read_idx - write_idx - 1)
                  : (LED_QUEUE_LEN - (write_idx - read_idx) - 1);

  if (space < SHORT_START_LEN + led_bits_len)
    return;

  for (i = 0; i < SHORT_START_LEN; i++) {
    led_queue[write_idx] = startseq[i];
    write_idx = NEXT_IDX(write_idx);
  }

  for (i = 0; i < 8; i++)
    msg_data[i] = (c >> i) & 1;
  encode_hamming_16_11(msg_type, msg_data, msg_bits);
  convert_to_led_bits(msg_bits, led_bits);

  for (i = 0; i < led_bits_len; i++) {
    led_queue[write_idx] = led_bits[i];
    write_idx = NEXT_IDX(write_idx);
  }
}

void enqueue_3char_led_bits(char c1, char c2, char c3) {
  int i;
  // Compute msg_type: increases every cycle_msg_type_every
  int msg_type[msg_type_len];
  // msg_type_len = 2,  4 values: 0, 1, 2, 3 for msg_type
  // each msg_type repeat cycle_msg_type times
  const int cycle_msg_type = 3;
  int group_id = (packet_count / cycle_msg_type) % 4;
  int msg_data[msg_data_len], msg_bits[msg_bits_len], led_bits[led_bits_len];
  int space = (read_idx > write_idx)
                  ? (read_idx - write_idx - 1)
                  : (LED_QUEUE_LEN - (write_idx - read_idx) - 1);

  msg_type[0] = group_id & 1;
  msg_type[1] = (group_id >> 1) & 1;

  if (space < SHORT_START_LEN + led_bits_len)
    return;

  // Add short start pattern
  for (i = 0; i < SHORT_START_LEN; i++) {
    led_queue[write_idx] = startseq[i];
    write_idx = NEXT_IDX(write_idx);
  }

  // Convert 3 ASCII characters to 24-bit msg_data
  for (i = 0; i < 8; i++)
    msg_data[i] = (c1 >> i) & 1;
  for (i = 0; i < 8; i++)
    msg_data[8 + i] = (c2 >> i) & 1;
  for (i = 0; i < 8; i++)
    msg_data[16 + i] = (c3 >> i) & 1;

  encode_hamming_32_26(msg_type, msg_data, msg_bits);
  convert_to_led_bits(msg_bits, led_bits);

  for (i = 0; i < led_bits_len; i++) {
    led_queue[write_idx] = led_bits[i];
    write_idx = NEXT_IDX(write_idx);
  }
}

// --- Timer ISR ---

void TIM1_UP_IRQHandler(void) {
  if (__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE) != RESET) {

    if (long_start_mode) {
      if ((long_start_counter % 2) == 0)
        LED_ON();
      else
        LED_OFF();
      long_start_counter++;

      if (long_start_counter >
          LONG_START_LEN) { // start pattern 10101..01, ends at 1
        long_start_mode = 0;
        long_start_counter = 0;
      }

    } else if (read_idx != write_idx) {
      if (led_queue[read_idx])
        LED_ON();
      else
        LED_OFF();
      read_idx = NEXT_IDX(read_idx);
    } else {
      LED_OFF(); // Must not happen
    }

    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
  }
}

// --- Main function ---

int main(void) {
  int i;
  HAL_Init();
  SystemClock_Config();
  SystemCLK_Init();
  My_GPIO_Init();
  My_Tim1_Init(PSC - 1, ARR - 1);
  HAL_TIM_Base_Start_IT(&htim1);

  // Preload n characters initially
  // n = 4*3, make sure this is longer than the full message
  for (i = 0; i < 4; i++) {
    // enqueue_char_led_bits(data_transmit[next_char++]);
    if (data_transmit[next_char + 2] != '\0') {
      enqueue_3char_led_bits(data_transmit[next_char],
                             data_transmit[next_char + 1],
                             data_transmit[next_char + 2]);
      packet_count++;
      next_char += 3;
    } else {
      // this shouldn't happen, make sure pre-buffering is shorter than the full
      // message
      next_char = 0;
      packet_count = 0;
    }
  }

  while (1) {
    int space = (read_idx > write_idx)
                    ? (read_idx - write_idx - 1)
                    : (LED_QUEUE_LEN - (write_idx - read_idx) - 1);

    while (!long_start_mode && space >= (SHORT_START_LEN + led_bits_len)) {
      // enqueue_char_led_bits(data_transmit[next_char++]);
      if (data_transmit[next_char + 2] != '\0') {
        enqueue_3char_led_bits(data_transmit[next_char],
                               data_transmit[next_char + 1],
                               data_transmit[next_char + 2]);

        next_char += 3;
      } else if (data_transmit[next_char + 1] != '\0') {
        // Only 2 characters left, reuse the 0th character
        enqueue_3char_led_bits(data_transmit[next_char],
                               data_transmit[next_char + 1], ' ');
        next_char = 0;
      } else {
        enqueue_3char_led_bits(data_transmit[next_char], ' ', ' ');
        next_char = 0;
      }
      packet_count++;
      space -= (SHORT_START_LEN + led_bits_len);
    }

    if (!long_start_mode &&
        (data_transmit[next_char + 2] == '\0' ||
         data_transmit[next_char + 1] == '\0' ||
         data_transmit[next_char] == '\0') &&
        read_idx == write_idx) { // Queue is empty
      next_char = 0;
      long_start_mode = 0; // Set to 1 if you want long preamble
      long_start_counter = 0;
    }
  }
}