lacam2
---
Test Only
This is a extended code repository of the paper ["Improving LaCAM for Scalable Eventually Optimal Multi-Agent Pathfinding"](https://kei18.github.io/lacam2/) (IJCAI-23).

## Building

```sh
cmake -B build && make -C build
```

## Usage

no optimization (random starts/goals):

```sh
> build/main -v 1 -m assets/random-32-32-20.map -N 400

makespan optimization:

```sh
> build/main -m assets/loop.map -i assets/loop.scen -N 3 -v 1 --objective 1
solved: 8ms     makespan: 10 (lb=2, ub=5)       sum_of_costs: 21 (lb=5, ub=4.2) sum_of_loss: 21 (lb=5, ub=4.2)
```

You can find details of all parameters with:
```sh
build/main --help
```

## Notes

- The grid maps and scenarios in `assets/` are from [MAPF benchmarks](https://movingai.com/benchmarks/mapf.html).
- `tests/` is not comprehensive. It was used in early developments.
- Auto formatting (clang-format) when committing:

```sh
git config core.hooksPath .githooks && chmod a+x .githooks/pre-commit
```

## Licence

This software is released under the MIT License, see [LICENSE.txt](LICENCE.txt).
