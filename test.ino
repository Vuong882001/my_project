#include "Arduino.h"
#include "coredecls.h"
#include <pgmspace.h>
#include "gdb_hooks.h"
#include "esp8266_peri.h"
#include "user_interface.h"
#include "uart_register.h"

#define UART0   0

void uart_init() {
    // Cấu hình UART0 với tốc độ baud là 9600
    WRITE_PERI_REG(UART_CLKDIV(UART0), UART_CLK_FREQ / 115200);
    // Tắt chế độ chẳng lẻ và chẳng chẵn (parity) trong thanh ghi UART_CONF0
    CLEAR_PERI_REG_MASK(UART_CONF0(UART0), UART_PARITY_EN | UART_PARITY | UART_BIT_NUM | UART_STOP_BIT_NUM);
    // Reset thanh ghi RX và TX
    SET_PERI_REG_MASK(UART_CONF0(UART0), UART_RXFIFO_RST | UART_TXFIFO_RST); // set len 1 de xoa bo nho fifo
    CLEAR_PERI_REG_MASK(UART_CONF0(UART0), UART_RXFIFO_RST | UART_TXFIFO_RST);// tra ve 0 mac dinh
}
void uart_send_char(uint8_t c) {
    // Đợi đến khi bộ đệm TX UART trống
     uint8_t fifo_cnt  = (READ_PERI_REG(UART_STATUS(UART0)) >> UART_TXFIFO_CNT_S) & UART_TXFIFO_CNT;
    // Check bộ nhớ có vượt quá 126 bytes không
      if(fifo_cnt < 126)
      {
        WRITE_PERI_REG(UART_FIFO(UART0), c);
      }
}
void uart_receive() {
  // kiem tra dữ liệu có trong rx fifo không
  uint8_t data;
  if (READ_PERI_REG(UART_STATUS(UART0)) & UART_RXFIFO_CNT) {
    // Đọc dữ liệu từ bộ đệm RX UART0
     data = READ_PERI_REG(UART_FIFO(UART0)) & 0xFF;
    // Xử lý dữ liệu nhận được tại đây
    // Ví dụ: In dữ liệu lên Serial Monitor
    Serial.write(data);
  }
    if(data='a'){
      digitalWrite(2,HIGH);
    }
    else digitalWrite(2,LOW);
}
void setup() {
    pinMode(2,OUTPUT);
    Serial.begin(115200); // Cài đặt tốc độ baud cho Serial Monitor
    uart_init();
}
void loop() {
    uart_receive();
}
