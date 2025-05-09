# Welcome to ZombieVerter contributing guide <!-- omit in toc -->

Thank you for investing your time in contributing to our Open Source project! 
Please keep in mind:
1. This is an open source project so no code should be used that does not work with this ethos
2. This is a large multifunctional software stack so any change you make needs to work with the rest of the stack
3. Final decision on new features and functions lie with the code owners. You can ofcourse spin off your own fork and do what you want
4. Due to the nature of the application of this code testing is critical, any un released software needs to be treated as unproven

In this guide you will get an overview of the contribution workflow from opening an issue, creating a PR, reviewing, and merging the PR.


## New contributor guide

To get an overview of the project, read the [README](../README.md) file. Here are some resources to help you get started with open source contributions:

- [Finding ways to contribute to open source on GitHub](https://docs.github.com/en/get-started/exploring-projects-on-github/finding-ways-to-contribute-to-open-source-on-github)
- [Set up Git](https://docs.github.com/en/get-started/git-basics/set-up-git)
- [GitHub flow](https://docs.github.com/en/get-started/using-github/github-flow)
- [Collaborating with pull requests](https://docs.github.com/en/github/collaborating-with-pull-requests)

## Getting started

### Compiling Linux
For writing code recommeded to use CodeBlocks IDE :  https://www.codeblocks.org/

You will need the arm-none-eabi toolchain: https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads
On Linux this can be installed by typing:

`sudo apt install gcc-arm-none-eabi`

The only external depedencies are libopencm3 and libopeninv. You can download and build this dependency by typing

`make get-deps`

Now you can compile stm32-vcu by typing

`make`

### Tests

Build the tests

`make Tests`

Run the tests

`./test/test_vcu`

And upload it to your board using a JTAG/SWD adapter, the updater.py script or the esp8266 web interface

### Compiling Windows

For writing code recommeded to use CodeBlocks IDE :  https://www.codeblocks.org/

Compiling in Docker using the information found here : https://github.com/crasbe/ZombieBuild

## Contributing and Pull Requests

There are 3 branches used in this project.

### master
The code that lives that has been validated on various vehicles and configurations in preperating for Release. 
Do not submit Pull Requests to this unless asked to do so by a Code Owner

### Vehicle_Testing 
The code that lives here is ready or ongoing testing and validationg on various vehicles and configurations.
Pull Requests will be accepted on the basis there has been some testing done already. 
When contributing best to build off of this branch unless you have specific reasons to use the branch Developmental_NOT_VEHICLE

### Developmental_NOT_VEHICLE 
The code that lives here is untested beyond the fact it compiles, any major changes or frameworks will be published here for review
Pull Requests will be accepted here if they 1. Pass Compile tests 2. Do not immediately break other features
When contributing you can build off of this branch if you have a specific need to do so

## Issues
These are used to track:
1.Feature requests
2.Bugs in code during development or in a release

### Create a new issue
Ensure you specify if this is a "Bug" or a "Request".

#### Feature
If it is a "Feature", label it as such and provide reasoning for this feature. Ensure to include:
1.How to implement this with the existing ZombieVerter VCU hardware
2.Required information for external interactions or communications

Note your feature can be rejected by the code owners if not inline with the ethos or intent of ZombieVerter

#### Bug
If you find a "Bug", label it as such and provide information regarding the found problem:
1.Provide Release version or Build number
2.Provide all information on your setup, prefferbly including pictures and diagrams
3.Provide a .Json of the parameters used
4.Description of the behaviour pattern, with supporting logs (CAN and/or Traces)
5.What behavriour you would be expecting to happen

Note your "Bug" might be due to behaviour working differently then you expect, it will be considered and reviewed by the code owners if not inline with the ethos or intent of ZombieVerter it will be commented on and closed.
