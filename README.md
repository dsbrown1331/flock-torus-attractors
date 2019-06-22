# flock-torus-attractors

This repository contains Matlab code for a multi-agent swarm simulation that has both a flock and torus attractor that result from the same model parameters. 

The code in `swarm_simulation.m` is the base model used in the papers [Supporting human interaction with robust robot swarms](https://www.cs.utexas.edu/~dsbrown/pubs/Kerman_RCS2012.pdf), [Human-swarm interactions based on managing attractors](https://www.cs.utexas.edu/~dsbrown/pubs/Brown_HRI2014.pdf), and [Limited bandwidth recognition of collective behaviors in bio-inspired swarms](https://www.cs.utexas.edu/~dsbrown/pubs/Brown_AAMAS2014.pdf).

The code in `StakeholderSwitching.m` is for exploring how to change the collective behavior of a swarm by influencing only a small subset of the members of the swarm. This code was used to generate the results for the stakeholder experiments in the paper [Human-swarm interactions based on managing attractors](https://www.cs.utexas.edu/~dsbrown/pubs/Brown_HRI2014.pdf).

If you find this code useful please consider citing the following:

```
@inproceedings{brown2014human,
  title={Human-swarm interactions based on managing attractors},
  author={Brown, Daniel S and Kerman, Sean C and Goodrich, Michael A},
  booktitle={Proceedings of the 2014 ACM/IEEE international conference on Human-robot interaction},
  pages={90--97},
  year={2014},
  organization={ACM}
}
```

```
@inproceedings{brown2014limited,
  title={Limited bandwidth recognition of collective behaviors in bio-inspired swarms},
  author={Brown, Daniel S and Goodrich, Michael A},
  booktitle={Proceedings of the 2014 international conference on Autonomous agents and multi-agent systems},
  pages={405--412},
  year={2014},
  organization={International Foundation for Autonomous Agents and Multiagent Systems}
}
```


