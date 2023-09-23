/** Include Header Files **/
#include "Queue_private.h"


/**
 * @description: 队列初始化
 * @param {None} 
 * @return: void
 * @note: 
 */ 
Queue queue_init(void)
{
    Queue q;
    q.front = q.rear = 0;
    return q;
}

/**
 * @description: 判断队列是否为空
 * @param q:队列指针 
 * @return: 1--队列为空 0--队列不空
 * @note: 
 */ 
uint8_t queue_empty(Queue* q)
{
    return q->front == q->rear;
}

/**
 * @description: 判断队列是否已满
 * @param q:队列指针 
 * @return: 1--队列已满 0--队列不满
 * @note: 
 */
uint8_t queue_full(Queue* q)
{
	 return (q->rear+1) % MAX_QUEUE_SIZE == (q->front);
}
/**
 * @description: 入队操作
 * @param ele：待入队元素 
 * @return: 0--队列已满 1--入队成功
 * @note: 
 */ 
uint8_t EnQueue(Queue *q, uint8_t ele)
{
    /* 队满 */
    if (queue_full(q))
        return 0;
 
    /* 入队 */
    q->array[q -> front] = ele;
    q->front = (q->front+1) % MAX_QUEUE_SIZE;
    return 1;
}


/**
 * @description: 出队操作
 * @param {None} 
 * @return: 0--队列已空 1--出队成功
 * @note: 返回出队数据
 */
uint8_t DeQueue(Queue *q)
{
	  uint8_t temp;
	
    /* 队空 */
    if(queue_empty(q))
        return 0;
 
    /* 出队 */
		temp = q->array[q->rear];
    q->rear=(q->rear+1) % MAX_QUEUE_SIZE;
		return temp;
}


/**
 * @description: 清空队列
 * @param {None} 
 * @return: 
 * @note: 
 */
void queue_clear(Queue *q)
{
    q->front = q->rear = 0;
}


/**
 * @description: 获取队列长度
 * @param {None} 
 * @return: 
 * @note: 
 */
uint8_t queue_len(Queue* q)
{
    return (q->front + MAX_QUEUE_SIZE - q->rear) % MAX_QUEUE_SIZE;
}
