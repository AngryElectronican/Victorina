#pragma once
#include <inttypes.h>
#include <avr/io.h>

#define SIZE 32
#define MASK SIZE-1
struct CircleBUF{
  volatile uint8_t head=0;
  volatile uint8_t tail=0;
  volatile uint8_t buf[SIZE];
  volatile uint8_t count=0;
};

void buf_push(CircleBUF* buffer, uint8_t data);
uint8_t buf_pull(CircleBUF* buffer);
uint8_t buf_available(CircleBUF* buffer);
