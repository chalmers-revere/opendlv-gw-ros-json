/*
 * Copyright (C) 2018 Ola Benderius
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

#ifndef INTERFACE_HPP
#define INTERFACE_HPP

#include <memory>
#include <string>

class Od4Ros {

 public:
  Od4Ros(uint16_t);
  Od4Ros(Od4Ros const &) = delete;
  Od4Ros &operator=(Od4Ros const &) = delete;
  ~Od4Ros();
  void toOd4(std::string const &);

 private:
  std::unique_ptr<cluon::OD4Session> m_od4Session;
};

#endif
