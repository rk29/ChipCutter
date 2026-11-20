# Information for humans

## Introduction

Hi, I'm Ray. I started this project. I'm a human (weird that I have to say that), and I have many years of
experience writing software.
This project was something different for me. I hope you, the reader, enjoy it, or find it useful, or at least
learn something from it.

## Motivation

I started this project for multiple reasons:
- I wanted to be able to do true 4-axis machining on my Carvera Air
- I wanted to see just how far modern agent-driven software implementation could go
- I wanted to increase my understanding of how to build complex software using a spec-driven approach.

I have many years of experience writing software of many types, and I'm sure that I could have written the
code for this myself, given a few years time.
Everything up to this point was generated in about three days using Antigravity (agy) and Gemini 4 Carbon and Neon.
I reviewed some of the code, but never changed any of it directly.

## Process

The process I used was simple:
- I did some research, and decided on rust+egui for the implementation, based on seeing a number
  of other projects that used that combination to good effect. I'm capable with rust, but it's not my strongest
  language.
- I asked agy to create a skeleton of the project. I set some ground rules for development, but not a lot.
- I asked agy to create a CAM program, and used the /grill-me command to answer a lot of questions.
- I asked agy to move the files into the project directory so I could capture the process.
- I then selected a few features to add to phase 1 (it called it "Phased v1" back then, I later had it rename these)
- I asked agy to generate milestones for phase 1, then one by one I asked it to create a plan for each milestone and then execute it.
  I made a few adjustments to these plans as we went, almost exclusively by asking agy to make the changes.
- Somewhere along the line I reworked my development environment so that agy could just push directly into my forgejo
  instance, and could poll for issues that I reported, fix them, and then push that code as well.
- When I finished each phase, I selected the next set of features for a phase and repeated the process.
  For the most part, agy did the work while I was away (at work) or other wise occupied. I only checked in on it
  a couple times per day, and then mostly just to tell it to do the next step.
- In between phase 1 and 2 I asked agy to suggest some refactoring, then adjusted the plan for that a bit and let agy do it.
  The code is probably due for another round of refactoring, but I am hopeful that agy picked up on the intent of that session
  and integrated it with the rest of the process. Only time will tell.

At the time of this writing, the program is almost 90K lines of rust, so it's still not all that large, but it's large enough that I'm convinced
that software development will never be the same.

Also at the time of this writing, I have only begun to test the gcode output on a real machine.
Please do not trust the output to move a sharp metal spinning shaft around without doing a lot of simulations (using other software) and dry-runs.
I plan to for sure.

## Open Source

I didn't "write" any of the code here, and I feel that just about anyone could have gotten similar results.
Releasing this as open source seems like the right thing to do, so that maybe someone else can benefit from it, even if just to learn.

After I checked with my employer, I found that it was ok for me to release this as open source, as long as it uses a "free and open" license.
They still own the copyright, but anyone can take the code, fork it, and do what they want with it.

## Goal check
- [X] I wanted to be able to do true 4-axis machining on my Carvera Air (untested, but the gcode simulation looks correct)
- [X] I wanted to see just how far modern agent-driven software implementation could go (it went all the way through a complex project, and it did it quickly)
- [X] I wanted to increase my understanding of how to build complex software using a spec-driven approach (I learned a whole lot about how to use agentic tools to create applications)

## Conclusion

Thanks for reading all of that.
It's been an interesting ride so far, and I'm looking forward to seeing where else this goes.
Once it's up on github, I may point my issue poller at it.
I'll definitely have agy look at any pull requests and give a summary of the changes at a minimum.

-Ray
2026-Nov-19
