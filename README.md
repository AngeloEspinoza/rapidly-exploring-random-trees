# Rapidly-exploring Random Trees

<p align="center">
  <img src="https://user-images.githubusercontent.com/40195016/182943757-d39d012b-6f05-41da-a3be-b8590cf1335d.gif" />
</p>

## Description
A 2D simulation in the framework Pygame of the paper [Rapidly-exploring random trees: A new tool for path planning](https://www.cs.csustan.edu/~xliang/Courses/CS4710-21S/Papers/06%20RRT.pdf).
The environment has 2 non-convex obstacles that can be used or not. It shows different elements of the tree while building it, like the $x_{new}$ or $x_{rand}$ nodes. Also,
the $\varepsilon$ and the maximum allowed nodes variables and be adjusted. 

## Usage
```
Implements the RRT algorithm for path planning.

options:
  -h, --help            show this help message and exit
  -o, --obstacles, --no-obstacles
                        Obstacles on the map
  -n , --nodes          Maximum number of nodes
  -e , --epsilon        Step size
  -init  [ ...], --x_init  [ ...]
                        Initial node position in X and Y respectively
  -goal  [ ...], --x_goal  [ ...]
                        Goal node position in X and Y respectively
  -srn, --show_random_nodes, --no-show_random_nodes
                        Show random nodes on screen
  -snn, --show_new_nodes, --no-show_new_nodes
                        Show new nodes on screen
```

## Examples
Generate obstacles in the map and show the random nodes $x_{rand}$ 

```python3 RRT.py --obstacles --show_random_nodes```

No obstacles, show the new nodes $x_{new}$

```python3 RRT.py --no-obstacles --show_new_nodes```

## License 
 MIT License

Copyright (c) [2022] [Angelo Espinoza]

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
