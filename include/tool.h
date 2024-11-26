#ifndef TOOL_H
#define TOOL_H

#include <iostream>
#include <string.h>
#include <termio.h>
#include <unistd.h>

using namespace std;

#define IDNUM 7


// extern "C"
// { // 添加extern "C"
    void Mcopy(float (*C)[4], float (*P)[4]);

    void show_value(string name, float (*T)[4]);
    void copy_value(float *copy, float *paste, int n);
    void show_value(string name, float *value);
    void show_value(string name,float* value,int n);
    void show_value(string name, uint32_t *value);
    void show_point(string name, float *value);
void show_value(string name, uint8_t *value);
    void get_cmdlist(uint8_t *L, uint8_t c);
    void get_paralist(uint32_t *L, uint32_t c);

    int getch();
    char scanKeyboard();

    // void test_demo();
    // void test();

// } // 添加extern "C"
#endif
