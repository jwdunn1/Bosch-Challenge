/*                                                                         80->|
 * timer.h
 *
 * Modifications: James William Dunn
 *          Date: August 31, 2017
 *          A timer class
 
MIT License

Copyright(c) 2017 James William Dunn

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files(the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/

#ifndef TIMER_H
#define TIMER_H

#include <chrono>

class Timer1 {
 public:
  using high_resolution_clock = std::chrono::high_resolution_clock;
  using ms = std::chrono::milliseconds;
  Timer1();
  ~Timer1();
  void start();
  void restart();
  ms duration();
  ms since();
  long sinceBeginning();

 private:
  high_resolution_clock::time_point start_;
  high_resolution_clock::time_point beginning_;
};


//////////////////////////////////////////////////////////////////////////////
// Timer1 class implementation

Timer1::Timer1() {
  start();
  beginning_ = start_;
}
Timer1::~Timer1() {}

void Timer1::start() {
  start_ = high_resolution_clock::now();
}

void Timer1::restart() {
  start();
  beginning_ = high_resolution_clock::now();
}

Timer1::ms Timer1::duration() {
  high_resolution_clock::time_point now = high_resolution_clock::now();
  Timer1::ms durat = std::chrono::duration_cast<ms>(now - start_);
  start_ = now;
  return durat;
}

Timer1::ms Timer1::since() {
  high_resolution_clock::time_point now = high_resolution_clock::now();
  Timer1::ms durat = std::chrono::duration_cast<ms>(now - start_);
  return durat;
}

long Timer1::sinceBeginning() {
  high_resolution_clock::time_point now = high_resolution_clock::now();
  Timer1::ms t = std::chrono::duration_cast<ms>(now - beginning_);
  return t.count();
}

#endif /* TIMER_H */