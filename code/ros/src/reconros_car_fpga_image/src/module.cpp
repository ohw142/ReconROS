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

#include "module.h"

Module::Module(bool hasSoftware):
  m_IsActive(false), m_HasSoftware(hasSoftware)
{
}

Module::~Module()
{
}

bool Module::hasSoftware()
{
  return m_HasSoftware;
}

void Module::switchToSoftware()
{
}

void Module::switchToHardware(int slot)
{
}

void Module::setIsActive(bool value)
{
  m_IsActive = value;
}

bool Module::isActive() {
  return m_IsActive;
}
