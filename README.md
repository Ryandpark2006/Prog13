# Prog13

Name: Ryan Park
EID: rwp749

Notes: 
Currently, my implementation is a vanilla 5-stage pipeline with forwarding. In terms of testing, I used the same tests that I had from last week's programming assignment. To repeat the tests that I made last time, along with the passing of all test cases on GradeScope: 

Testing: 
The file called tinker_tb.sv has a series of tests for 26 instructions that were tested in the previous progs. The majority of the instructions that 
involve assigning or calculating the program counter are tested by getting the program counter and seeing if the value is equal to the expected value 
through a conditional statement. My testing document waits 10 clock cycles before checking if the values are what we expect. The wait times 
between each test case is 5 clock cycles to allow the reset to propogate throughout and set each control and data path to its default values. 

Optimizations: 
Forwarding 

