#ifndef __QUEUE_H_
#define __QUEUE_H_

#include "main.h"

/* 队的最大长度 */
#define MAX_QUEUE_SIZE 10


typedef struct {
	
    uint8_t array[MAX_QUEUE_SIZE];  
    int front;	/* 队头 */   
    int rear;	/* 队尾 */
	
}Queue;

Queue queue_init(void);
uint8_t queue_empty(Queue* q);
uint8_t EnQueue(Queue *q, uint8_t ele);
uint8_t DeQueue(Queue *q);
void queue_clear(Queue *q);
uint8_t get_front(Queue* q);
uint8_t queue_len(Queue* q);

#endif
