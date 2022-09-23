#include <iostream>
#include <altro/augmented_lagrangian/al_solver.hpp>

int main() {
  const int NumStates = 4;
  const int NumControls = 2;
  int num_segments = 100;
  altro::augmented_lagrangian::AugmentedLagrangianiLQR<NumStates, NumControls> solver(num_segments);

  return 0;
}