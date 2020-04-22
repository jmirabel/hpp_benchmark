#include <vector>
#include <map>

namespace hpp {
namespace benchmark {

typedef double value_type;
typedef std::vector<value_type> result_t;
typedef std::map<std::string, result_t> results_t;

class BenchmarkCase {
  public:
    virtual ~BenchmarkCase () {}

    virtual void setup() = 0;

    virtual void initializeProblem() = 0;

    virtual void solveProblem() = 0;

    virtual void saveResolutionResult(results_t& results) = 0;

    /// Be default, the solution is considered valid.
    virtual bool validateSolution ()
    {
      return true;
    }

    virtual void clean() = 0;
}; // class Resolution

void registerBenchmark(BenchmarkCase* p, const std::string& name);
} // namespace benchmark
} // namespace hpp

#define REGISTER(CLASS, ID)                                                    \
struct __registerer__ ## ID {                                                  \
  __registerer__ ## ID () {                                                    \
    ::hpp::benchmark::registerBenchmark(new CLASS(), #ID);                     \
  }                                                                            \
};                                                                             \
__registerer__ ## ID __registered_instance__ ## ID
