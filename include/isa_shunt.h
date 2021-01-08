#ifndef SimpleISA_h
#define SimpleISA_h

/*  This library supports the ISA Scale IVT Modular current/voltage sensor device.  These devices measure current, up to three voltages, and provide temperature compensation.
    This library was written by Jack Rickard of EVtv - http://www.evtv.me copyright 2016
    You are licensed to use this library for any purpose, commercial or private,
    without restriction.

*/

#include <stdint.h>
#include "my_fp.h"

class ISA
{

    ISA();
    ~ISA();



public:

    static		void initialize();
    static		void initCurrent();
    static		void sendSTORE();
    static		void STOP();
    static		void START();
    static		void RESTART();
    static		void deFAULT();

    static		int32_t Voltage;
    static		int16_t Temperature;
    static      int32_t Amperes;   // Floating point with current in Amperes
    static      int32_t KW;


    static void handle521(int id, uint32_t data[2], uint32_t time);
    static void handle522(int id, uint32_t data[2], uint32_t time);
    static  void handle523(int id, uint32_t data[2], uint32_t time);
    static  void handle524(int id, uint32_t data[2], uint32_t time);
    static void handle525(int id, uint32_t data[2], uint32_t time);
    static void handle526(int id, uint32_t data[2], uint32_t time);
    static void handle527(int id, uint32_t data[2], uint32_t time);
    static void handle528(int id, uint32_t data[2], uint32_t time);





private:

    static		unsigned long elapsedtime;
    static		double  ampseconds;
    static		int milliseconds ;
    static		int seconds;
    static		int minutes;
    static		int hours;
    static		char buffer[9];
    static		char bigbuffer[90];




};

#endif /* SimpleISA_h */
