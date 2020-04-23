#include <vector>
#include <map>

namespace hpp {
namespace benchmark {

typedef double value_type;
typedef std::vector<value_type> result_t;
typedef std::map<std::string, result_t> results_t;

class BenchmarkRunner;

class BenchmarkBase {
  public:
    virtual ~BenchmarkBase () {}

    virtual void setup(int Niter) = 0;

    virtual void clean() = 0;

  protected:
    /// You should never call or reimplement this yourself, unless you know what
    /// you do.
    virtual void run(int N, const std::string& name) = 0;

    friend class BenchmarkRunner;
}; // class Resolution

class BenchmarkCase : public BenchmarkBase {
  public:
    virtual ~BenchmarkCase () {}

    virtual void initializeProblem(int iIter) = 0;

    virtual void solveProblem() = 0;

    virtual void saveResolutionResult(results_t& results) = 0;

    /// Be default, the solution is considered valid.
    virtual bool validateSolution ()
    {
      return true;
    }

  private:
    void run(int N, const std::string& name);
}; // class Resolution

class BenchmarkNCase : public BenchmarkBase {
  public:
    virtual ~BenchmarkNCase () {}

    virtual std::vector<std::string> names () = 0;

    virtual void initializeProblem(int iIter, int iCase) = 0;

    virtual void solveProblem() = 0;

    virtual void saveResolutionResult(results_t& results) = 0;

    /// Be default, the solution is considered valid.
    virtual bool validateSolution ()
    {
      return true;
    }

    virtual void clean() = 0;

  private:
    void run(int N, const std::string& name);
}; // class Resolution

void registerBenchmark(BenchmarkBase* p, const std::string& name);
} // namespace benchmark
} // namespace hpp

#define REGISTER(CLASS, ID)                                                    \
struct __registerer__ ## ID {                                                  \
  __registerer__ ## ID () {                                                    \
    ::hpp::benchmark::registerBenchmark(new CLASS(), #ID);                     \
  }                                                                            \
};                                                                             \
__registerer__ ## ID __registered_instance__ ## ID
