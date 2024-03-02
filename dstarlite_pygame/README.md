# d-star-lite

Implementation based off of Sven Koenig and Maxim Likhachev's original D* Lite paper http://idm-lab.org/bib/abstracts/papers/aaai02b.pdf

We modified code from https://github.com/mdeyo/d-star-lite for an 8-way connected grid to match the code we wrote for our ROS implementation.

Instructions for installing pygame can be found at [https://www.pygame.org/wiki/GettingStarted](https://www.pygame.org/wiki/GettingStarted).

Run the example demo with ```python3 main.py```

In the demo, the blue circle represents the robot and the user sets the goal location by clicking a grid cell. To add obstacles, click on free grid cells. The robot moves and rescans every time the spacebar is pressed.
