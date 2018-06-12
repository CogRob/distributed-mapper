# Distributed Mapper Docker README

This is a docker container for distributed_mapper. 

The latest docker container can be pulled from DockerHub by running: 

```
docker pull awilby/distributed_mapper:latest
```

Then, run Distributed Mapper in the docker container by running: 

```
docker run distributed_mapper --nrRobots <num_robots> --dataDir <data_directory>
```

For example, to run Distributed Mapper on example data using 4 robots, run:

```
docker run distributed_mapper --nrRobots 4 --dataDir ../../data/example_4robots/
```
