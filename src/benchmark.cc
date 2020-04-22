#include "benchmark.hh"

#include <cstring>
#include <iostream>
#include <math.h>
#include <hpp/util/timer.hh>

typedef std::map<std::string, hpp::benchmark::BenchmarkCase*> Problems_t;
Problems_t problems;

namespace hpp {
namespace benchmark {

void registerBenchmark(BenchmarkCase* p, const std::string& name)
{
  if (!problems.insert(std::make_pair(name, p)).second) {
    std::invalid_argument("Problem " + name + " already defined.");
    delete p;
  }
}

void stats (result_t values, value_type& mean, value_type& std_dev)
{
  mean = 0;
  std_dev = 0;
  int n = 0;
  for (auto& v : values) {
    if (!isnan(v) && !isinf(v)) {
      mean += v;
      std_dev += v*v;
      ++n;
    }
  }
  mean /= n;
  std_dev /= n;
  std_dev = std::sqrt(std_dev - mean*mean);
}

} // namespace benchmark
} // namespace hpp

void print_header(std::ostream& os, const std::string& status)
{
  int size = 10;
  int l = static_cast<int>(status.size());
  int before = (size - l) / 2;
  int after  = size - l - before;
  os << '[';
  for (int i = 0; i < before; ++i) os << ' ';
  os << status;
  for (int i = 0; i < after ; ++i) os << ' ';
  os << ']' << ' ';
}

void runBenchmark(int N, const std::string& name, hpp::benchmark::BenchmarkCase* problem)
{
  using hpp::benchmark::value_type;
  hpp::debug::Timer timer;

  print_header(std::cout, "RUN");
  std::cout << "Setup " << name << ' ' << std::flush;
  problem->setup();

  hpp::benchmark::results_t results;
  for (int i = 0; i < N; ++i) std::cout << '.';
  for (int i = 0; i < N; ++i) std::cout << '\b';
  for (int i = 0; i < N; ++i) {
    problem->initializeProblem();
    timer.start();
    problem->solveProblem();
    timer.stop();
    results["Time (s)"].push_back(1e-3 * static_cast<value_type>(timer.duration().total_milliseconds()));
    problem->saveResolutionResult(results);
    if (problem->validateSolution())
      std::cout << '|' << std::flush;
    else
      std::cout << '-' << std::flush;
  }

  problem->clean();
  std::cout << std::endl;

  hpp::benchmark::value_type mean, std_dev;
  for (auto& result : results) {
    print_header(std::cout, "Result");
    hpp::benchmark::stats(result.second, mean, std_dev);
    std::cout << result.first << '\t' << mean << " +/- " << std_dev << '\n';
  }
}

void usage(const char* name)
{
  std::cout << name
    << " --help        show usage and exit\n"
    << " --list        list benchmarks and exit\n"
    << " --run <name>  run only one benchmark. All options after this are ignored.\n"
    << " -N <integer>  set the number of repetition for a benchmark\n"
    << std::flush;
}

int main(int argc, char**argv)
{
  int N = 20;
  int iarg = 0;
  // Parse arguments
  while (++iarg < argc) {
    if (strcmp(argv[iarg], "--help") == 0) {
      usage(argv[0]);
      return 0;
    } else if (strcmp(argv[iarg], "--list") == 0) {
      std::cout << "Available benchmarks are:\n";
      for (auto& pair : problems)
        std::cout << pair.first << '\n';
      return 0;
    } else if (strcmp(argv[iarg], "--run") == 0) {
      std::string name (argv[++iarg]);
      if (problems.count(name) == 0) {
        std::cerr << name << " not found.\n";
        return 1;
      }
      runBenchmark(N, name, problems[name]);
      return 0;
    } else if (strcmp(argv[iarg], "-N") == 0) {
      N = (int)strtol(argv[++iarg],NULL,0);
    }
  }

  // Run the benchmarks
  for (auto& pair : problems)
    runBenchmark(N, pair.first, pair.second);
  return 0;
}
