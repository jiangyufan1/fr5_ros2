// Offline test adapter; this library does not use ROS or a robot SDK.
#define IKFAST_NO_MAIN
#include "fr5_ikfast_solver.cpp"
#include <algorithm>
#include <vector>

static_assert(sizeof(IkReal) == sizeof(double), "Expected double precision IKFast");
extern "C" int fr5_joint_count() { return GetNumJoints(); }
extern "C" int fr5_free_count() { return GetNumFreeParameters(); }
extern "C" int fr5_fk(const double* q, double* t, double* r) {
  try { ComputeFk(q, t, r); return 0; } catch (...) { return -1; }
}
extern "C" int fr5_ik(const double* t, const double* r, double* output, int capacity) {
  try {
    ikfast::IkSolutionList<IkReal> solutions;
    if (!ComputeIk(t, r, nullptr, solutions)) return 0;
    if (solutions.GetNumSolutions() > static_cast<size_t>(capacity)) return -2;
    for (size_t i = 0; i < solutions.GetNumSolutions(); ++i) {
      const auto& solution = solutions.GetSolution(i);
      std::vector<IkReal> free_values(solution.GetFree().size(), 0.0);
      solution.GetSolution(output + i * 6,
                           free_values.empty() ? nullptr : free_values.data());
    }
    return static_cast<int>(solutions.GetNumSolutions());
  } catch (...) { return -1; }
}
