# hpp_benchmark

A set of benchmark to track the evolution of performances of HPP.

## Running the C++ benchmarks

Compile target `benchmarks` and run
```cpp
benchmarks --output {source_dir}/results/cpp/{year}/{month}/{day}
```
Option `--output <dir>` can be omitted if you do not want to save the results to the disk.
See `benchmarks --help` for more usage.

### Save the results
The script will generate a bunch of csv files in the output directory (see `--output`).
To save the benchmarks, commit the output directory.

## Running the Python benchmarks

Start `hppcorbaserver` in a terminal. In another terminal, go in each sub-directories of
`future` and run the python script `script.py`.

### Save the results
Save the output of each script.py in a file called `benchmark` next to the script.
Then copy the directory `future` to `year-month-day` and commit this new directory.
