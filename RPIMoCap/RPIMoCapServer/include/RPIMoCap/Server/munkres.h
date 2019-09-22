/*
 *   Copyright (c) 2007 John Weaver
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

#pragma once

#include "matrix.h"

#include <list>
#include <vector>
#include <utility>

class Munkres {
public:
  void solve(Matrix<double> &m);
private:
  static constexpr int NORMAL = 0;
  static constexpr int STAR   = 1;
  static constexpr int PRIME  = 2;
  inline bool find_uncovered_in_matrix(const double, size_t&, size_t&) const;
  inline bool pair_in_list(const std::pair<size_t,size_t> &, const std::list<std::pair<size_t,size_t> > &);
  int step1();
  int step2();
  int step3();
  int step4();
  int step5();
  int step6();

  Matrix<int> mask_matrix;
  Matrix<double> matrix;
  bool *row_mask;
  bool *col_mask;
  size_t saverow, savecol;
};

// Set of functions for two-dimensional std::vector.
template <typename T>
Matrix <T> convert_std_2d_vector_to_munkres_matrix (const std::vector <std::vector <T> > & vector)
{
  const int dimention1 = vector.size();
  const int dimention2 = vector[0].size();
  Matrix <T> matrix (dimention1, dimention2);
  for (int i = 0; i < dimention1; ++i) {
    for (int j = 0; j < dimention2; ++j) {
      matrix (i, j) = vector [i][j];
    }
  }

  return matrix;
}

template <typename T>
void fill_std_2d_vector_from_munkres_matrix (std::vector <std::vector <T> > & vector, const Matrix <T> & matrix)
{
    const int dimention1 = vector.size();
    const int dimention2 = vector[0].size();
    for (int i = 0; i < dimention1; ++i) {
      for (int j = 0; j < dimention2; ++j) {
        vector[i][j]= matrix(i,j);
      }
    }
}

