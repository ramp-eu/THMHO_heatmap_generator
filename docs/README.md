# Temporal Heat Map for Human Occupancy - Heatmap generation

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Docker badge](https://img.shields.io/docker/pulls/ramp-eu/TTE.project1.svg)](https://hub.docker.com/r/<org>/<repo>/)
<br/>

[![Documentation Status](https://readthedocs.org/projects/thmho-lidar-node/badge/?version=latest)](https://thmho-heatmap-generator.readthedocs.io/en/latest/?badge=latest)
[![CI](https://github.com/ramp-eu/TTE.project1/workflows/CI/badge.svg)](https://github.com/ramp-eu/TTE.project1/actions?query=workflow%3ACI)
[![Coverage Status](https://coveralls.io/repos/github/ramp-eu/TTE.project1/badge.svg?branch=master)](https://coveralls.io/github/ramp-eu/TTE.project1?branch=master)
[![Codacy Badge](https://app.codacy.com/project/badge/Grade/fce8e4a4dfe140bb9963b88aaf1a2b03)](https://www.codacy.com/gh/ramp-eu/THMHO_heatmap_generator/dashboard?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=ramp-eu/THMHO_heatmap_generator&amp;utm_campaign=Badge_Grade)
[![CII Best Practices](https://bestpractices.coreinfrastructure.org/projects/5132/badge)](https://bestpractices.coreinfrastructure.org/projects/5132)

This project is part of [DIH^2](http://www.dih-squared.eu/). For more information check the RAMP Catalogue entry for the
[components](https://github.com/ramp-eu).

| :books: [Documentation](https://thmho-lidar-node.readthedocs.io/en/latest/README.html) | :whale: [Docker Hub](https://hub.docker.com) |
| --------------------------------------------- | ------------------------------------------------------------- |

## Contents

-   [Background](#background)
-   [Install](#install)
-   [Usage](#usage)
-   [API](#api)
-   [Testing](#testing)
-   [Feedback](#feedback)
-   [Contribution](#contribution)
-   [License](#license)

## Background
### Objective
The objective of the package is to create the temporal heatmaps that show for each 1 m2, and each 1 hour of each workday the rate of human occupancy.

## Install

Information about how to install the `THMHO_heatmap_generator` can be found at the corresponding section of the [Installation & Administration Guide](docs/installationguide.md).

## Usage

Information about how to use the `THMHO_heatmap_generator` can be found in the [User & Programmers Manual](docs/usermanual.md).

## Testing

### Troubleshooting

- Message `Requesting the map...` on terminal or `No map received` in RViz

    The file pointed by `sim_map_file` (for simulation) or `real_map_file` (for real laser scanners) in `factory_sim/launch/factory.launch` may not exist in the workspace.

- No heatmap is vizualized in `RViz`

    If `Costmap` is not visualized as well, verify if the laser scan `topic` names specified in `factory_sim/config/costmap_params.yaml` are correct, and if the `tf` tree is published correctly.
    If `Costmap` is visualized, it should be noted that the first heatmap is published after the time specified by `aggregation_time` argument for `heatmap_generator` node in `factory_sim/launch/factory.launch`. The default value is 3600, i.e. 1 hour. 


## Feedback

Any feedback and suggestions can be submitted by creating New issue in the Issues tab or by emailing the team.

## Contribution

In order to contribute you will have to request to be added to the project.

## License

The project is licensed under the [Apache-2](https://opensource.org/licenses/Apache-2.0) license.
