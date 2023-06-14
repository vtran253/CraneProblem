///////////////////////////////////////////////////////////////////////////////
// cranes_algs.hpp
//
// Algorithms that solve the crane unloading problem.
//
// All of the TODO sections for this project reside in this file.
//
// This file builds on crane_types.hpp, so you should familiarize yourself
// with that file before working on this file.
//
///////////////////////////////////////////////////////////////////////////////

#pragma once

#include <math.h>

#include <cassert>
#include <stack>

#include "cranes_types.hpp"

namespace cranes {

// Solve the crane unloading problem for the given grid, using an exhaustive
// optimization algorithm.
//
// This algorithm is expected to run in exponential time, so the grid's
// width+height must be small enough to fit in a 64-bit int; this is enforced
// with an assertion.
//
// The grid must be non-empty.
path crane_unloading_exhaustive(const grid& setting) {
    // grid must be non-empty.
    assert(setting.rows() > 0);
    assert(setting.columns() > 0);

    // Compute maximum path length, and check that it is legal.
    const size_t max_steps = setting.rows() + setting.columns() - 2;
    assert(max_steps < 64);

    path best_path(setting);
    for (size_t steps = 0; steps <= max_steps; steps++) {
        size_t pow_set_size = pow(2, steps);
        for (size_t counter = 0; counter < pow_set_size; counter++) {
            path current_path(setting);
            bool is_valid_path = true;
            for (size_t k = 0; k < steps; k++) {
                step_direction direction;

                int bit = (counter >> k) & 1;

                if (bit == STEP_DIRECTION_EAST) {
                    direction = STEP_DIRECTION_EAST;
                } else {
                    direction = STEP_DIRECTION_SOUTH;
                }
                if (current_path.is_step_valid(direction)) {
                    current_path.add_step(direction);
                } else {
                    is_valid_path = false;
                    break;
                }
            }
            if (is_valid_path) {
                if (current_path.total_cranes() > best_path.total_cranes()) {
                    best_path = current_path;
                }
            }
        }
    }
    return best_path;
}

// Solve the crane unloading problem for the given grid, using a dynamic
// programming algorithm.
//
// The grid must be non-empty.
// path crane_unloading_dyn_prog(const grid& setting) {
path crane_unloading_dyn_prog(const grid& setting) {
    // grid must be non-empty.
    assert(setting.rows() > 0);
    assert(setting.columns() > 0);

    // crane_grid: the grid of max crane number for corresponding position
    std::vector<std::vector<int>> crane_grid(setting.rows(), std::vector<int>(setting.columns(), 0));

    // trace back starting point, default the most right bottom block
    int trace_back_start_row = crane_grid.size() - 1;
    int trace_back_start_column = crane_grid[0].size() - 1;

    // records of the max crane number and it's position
    int max_crane_row = 0, max_crane_column = 0;
    int max_crane_val = -1;

    // main loop to generate a grid with maximum crane number of it's corresponding
    // input grid position
    for (size_t r = 0; r < setting.rows(); r++) {
        for (size_t c = 0; c < setting.columns(); c++) {
            if (r == 0 && c == 0) {
                continue;
            }
            // G[i][j] = None if G[i][j]==X
            if (setting.get(r, c) == CELL_BUILDING) {
                crane_grid[r][c] = -1;
                continue;
            }

            // from_above = None if i=0 or G[i-1][j]==X; or G[i-1][j] + [↓] otherwise
            int from_above;
            if (r == 0 || setting.get(r - 1, c) == CELL_BUILDING) {
                from_above = -1;
            } else {
                from_above = crane_grid[r - 1][c];
                int crane = setting.get(r, c) == CELL_CRANE ? 1 : 0;
                from_above = from_above == -1 ? -1 : from_above + crane;
            }

            // from_left = None if j=0 or G[i][j-1]==X; or G[i][j-1] + [→] otherwise
            int from_left;
            if (c == 0 || setting.get(r, c - 1) == CELL_BUILDING) {
                from_left = -1;
            } else {
                from_left = crane_grid[r][c - 1];
                int crane = setting.get(r, c) == CELL_CRANE ? 1 : 0;
                from_left = from_left == -1 ? -1 : from_left + crane;
            }

            if (from_above == -1 && from_left == -1) {
                crane_grid[r][c] = -1;
            } else if (from_above == -1) {
                crane_grid[r][c] = from_left;
            } else if (from_left == -1) {
                crane_grid[r][c] = from_above;
            } else {
                crane_grid[r][c] = from_above > from_left ? from_above : from_left;
            }
            if (crane_grid[r][c] > max_crane_val) {
                max_crane_val = crane_grid[r][c];
                max_crane_row = r;
                max_crane_column = c;
            }
        }
    }

    // if the endpoint is less than maximum crane, start trace back from the block with
    // maximum crane number
    if (crane_grid[trace_back_start_row][trace_back_start_column] < max_crane_val) {
        trace_back_start_row = max_crane_row;
        trace_back_start_column = max_crane_column;
    }

    // collection of steps in tracing back process
    std::stack<step_direction> step_trace_back;

    // trace back from the block, steps will be reverse order
    while (trace_back_start_row >= 0 && trace_back_start_column >= 0) {
        if (trace_back_start_row == 0 && trace_back_start_column == 0) {
            break;
        }
        int left_val = -1;
        int top_val = -1;
        if (trace_back_start_column > 0) {
            left_val = crane_grid[trace_back_start_row][trace_back_start_column - 1];
        }
        if (trace_back_start_row > 0) {
            top_val = crane_grid[trace_back_start_row - 1][trace_back_start_column];
        }
        if (left_val > top_val) {
            step_trace_back.push(STEP_DIRECTION_EAST);
            trace_back_start_column--;
        } else {
            step_trace_back.push(STEP_DIRECTION_SOUTH);
            trace_back_start_row--;
        }
    }

    // add the best path to return object
    path best(setting);
    while (!step_trace_back.empty()) {
        best.add_step(step_trace_back.top());
        step_trace_back.pop();
    }

    // compare input, crane grid, and output grid
    // std::cout << "\n\n\n\n\n\ninput grid: \n";
    // setting.print();
    // std::cout << "\n\n";
    // std::cout << "crane grid: \n";
    // for (auto&& r : crane_grid) {
    //     for (auto&& c : r) {
    //         std::cout << c << " ";
    //     }
    //     std::cout << "\n";
    // }
    // std::cout << "\n\n";
    // std::cout << "result grid: \n";
    // best.print();
    // std::cout << "========================\n";
    return best;
}

}  // namespace cranes
