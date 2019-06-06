/*
 * queue.h
 *
 * Created: 6/4/2019 4:21:59 AM
 *  Author: Nathan Brennan
 */ 


#ifndef QUEUE_H_
#define QUEUE_H_

#define QUEUE_SIZE 4

#include <stdio.h>


typedef struct Queue {
    unsigned char buf[QUEUE_SIZE];
    unsigned char cnt;
} Queue;

void QueueInit(Queue *Q) {
    (*Q).cnt = 0;
}

unsigned char QueueFull(Queue Q) {
    return (Q.cnt == QUEUE_SIZE);
}

unsigned char QueueEmpty(Queue Q) {
    return (Q.cnt == 0);
}

void QueuePrint(Queue Q) {
    int j;
    printf("Queue contents: \r\n");
    for (j = 0; j < QUEUE_SIZE; j++) {
        printf("Item %d", j);
        printf(": %d\r\n", Q.buf[j]);
    }
}

void QueuePush(Queue *Q, unsigned char item) {
    if (!QueueFull(*Q)) {
        //DisableInterrupts();
        (*Q).buf[(*Q).cnt] = item;
        (*Q).cnt++;
        //EnableInterrupts();
    }
}

unsigned char QueuePop(Queue *Q) {
    int i;
    unsigned char item = 0;
    if (!QueueEmpty(*Q)) {
        //DisableInterrupts();
        item = (*Q).buf[0];
        (*Q).cnt--;
        for (i = 0; i < (*Q).cnt; i++) {
            // shift fwd
            (*Q).buf[i] = (*Q).buf[i+1];
        }
        //EnableInterrupts();
    }
    return(item);
}

void QueueClear(Queue *Q) {
    while (!QueueEmpty(*Q)) {
        QueuePop(Q);
    }
}


#endif /* QUEUE_H_ */