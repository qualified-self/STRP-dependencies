/*
 * PqExtra.cpp
 *
 * (c) 2015 Sofian Audry        :: info(@)sofianaudry(.)com
 * (c) 2015 Thomas O Fredericks :: tof(@)t-o-f(.)info
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "PqExtra.h"


SquareOsc::SquareOsc(float period, float dutyCycle) {
  setPeriod(period);
  setDutyCycle(dutyCycle);
}

void SquareOsc::setup() {
  _startTime = millis();
}

void SquareOsc::update() {
  // Check where we are.
  _isOn = ((millis() - _startTime) % _period < _dutyCyclePeriod);
}

void SquareOsc::setPeriod(float period) {
  // Convert period in ms.
  period *= 1000;
  _period = round(period);
  _period = max(_period, 1.0f); // at least 1ms
}

void SquareOsc::setDutyCycle(float dutyCycle) {
  // Convert duty cycle in ms.
  dutyCycle = constrain(dutyCycle, 0, 1);
  dutyCycle *= _period;
  _dutyCyclePeriod = round(dutyCycle);
}

SerialOut::SerialOut(uint8_t digits) : _digits(digits) {}

float SerialOut::put(float value) {
  // Copy value.
  __value = value;

  // Print it.
  println(__value, _digits);

  // Return it.
  return __value;
}


OscilloscopeOut::OscilloscopeOut(float minValue, float maxValue, uint8_t precision)
  : _minValue(minValue), _maxValue(maxValue), _precision(precision) {}

float OscilloscopeOut::put(float value) {
  // Copy value.
  __value = value;

  // Convert to bin.
  float mapped = map(__value, _minValue, _maxValue, 0.0f, 1.0f);
  int bin = round( mapped * _precision );
  bin = constrain(bin, 0, _precision-1);

  // Print.
  print(_minValue, 2);
  print(" |");
  for (int i=0; i<_precision; i++)
    print(i == bin ? '*' : ' ');
  print("| ");
  print(_maxValue, 2);
  println();

  // Return it.
  return __value;
}

Smoother::Smoother(float factor)
  : PqPutter(),
    MovingAverage(factor) {
}

float Smoother::put(float value) {
  return MovingAverage::update(value);
}

AdaptiveNormalizer::AdaptiveNormalizer(float smoothFactor)
  : PqPutter(),
    MovingStats(smoothFactor),
    __value(0.5f),
    _mean(0.5f),
    _stddev(0.25f)
{}

AdaptiveNormalizer::AdaptiveNormalizer(float mean, float stddev, float smoothFactor)
	: PqPutter(),
    MovingStats(smoothFactor),
    __value(mean),
    _mean(mean),
    _stddev(abs(stddev))
{}

float AdaptiveNormalizer::put(float value) {
  return (__value = MovingStats::update(value) * _stddev + _mean);
}

Normalizer::Normalizer()
  : PqPutter(),
    SimpleStats(),
    __value(0.5f),
    _mean(0.5f),
    _stddev(0.25f)
{}

Normalizer::Normalizer(float mean, float stddev)
	: PqPutter(),
    SimpleStats(),
    __value(mean),
    _mean(mean),
    _stddev(abs(stddev))
{}

float Normalizer::put(float value) {
  return (__value = SimpleStats::update(value) * _stddev + _mean);
}

MinMaxScaler::MinMaxScaler()
 : PqPutter(),
   __value(0.5f),
   __min(FLT_MAX),
   __max(FLT_MIN)
{}

float MinMaxScaler::put(float value)
{
  __min = min(value, __min);
  __max = max(value, __max);
  __value = map(value, __min, __max, 0.0f, 1.0f);
}

Thresholder::Thresholder(float threshold, uint8_t mode)
  : PqPutter(),
    _threshold(threshold),
    _mode(mode),
    _prev(0),
    __value(0) {
}

float Thresholder::put(float value) {
  bool high = (value > _threshold);
  bool low  = (value < _threshold);
  bool raising = (high && _prev != (+1));
  bool falling = (low  && _prev != (-1));
  switch (_mode) {
    case THRESHOLD_HIGH:    __value = high;    break;
    case THRESHOLD_LOW:     __value = low;     break;
    case THRESHOLD_RISING:  __value = raising; break;
    case THRESHOLD_FALLING: __value = falling; break;
    case THRESHOLD_CHANGE:
    default:                __value = raising || falling;
  }
  _prev = (value < _threshold ? (-1) : (value > _threshold ? (+1) : (0)));
  return (float)(__value ? 1 : 0);
}
