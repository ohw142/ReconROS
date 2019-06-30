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

#ifndef STATE_H
#define STATE_H
#include <atomic>

class State {
public:
  explicit State();
  bool running();
  bool paused();
  char mode();
  int throttle();
  int steering();
  void setRunning(bool value);
  void setPaused(bool value);
  void setMode(char value);
  void setThrottle(int value);
  void setSteering(int value);
private:
  std::atomic<bool> m_Running;
  std::atomic<bool> m_Paused;
  std::atomic<char> m_Mode;

  std::atomic<int> m_Throttle;
  std::atomic<int> m_Steering;

};

#endif //STATE_H
