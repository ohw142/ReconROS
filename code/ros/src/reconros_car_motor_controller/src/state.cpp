/*
 * ReconROS 
 * Copyright (C) 2019  Andreas Krakau, Felix Paul Jentzsch
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
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include "state.h"

State::State() :
  m_Running(false), m_Paused(false), m_Mode(0), m_Steering(0), m_Throttle(0)
{}

bool State::running() {
  return m_Running.load();
}

bool State::paused() {
  return m_Paused.load();
}

char State::mode() {
  return m_Mode.load();
}

void State::setRunning(bool value) {
  m_Running.store(value);
}

void State::setPaused(bool value) {
  m_Paused.store(value);
}

void State::setMode(char value) {
  m_Mode.store(value);
}

int State::throttle() {
  return m_Throttle.load();
}

int State::steering() {
  return m_Steering.load();
}

void State::setThrottle(int value) {
  m_Throttle.store(value);
}

void State::setSteering(int value) {
  m_Steering.store(value);
}
