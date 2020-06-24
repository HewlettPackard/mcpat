# McPAT

McPAT: (**M**)ulti(**c**)ore (**P**)ower, (**A**)rea, and (**T**)iming
Current version 1.4.0

## About McPAT
What McPAT is:
- Architectural integrated power, area, and timing modeling framework, focuses on power and area modeling, with a target clock rate as a design constraint. 
  - Consider power, area, and timing simultaneously 
  - Complete power envelope
  - Power management techniques
- Manycore processor modeling framework
  - Different cores, uncore, and system (I/O) components
  - Holistically modeling across stacks: Technology models from ITRS projections (also supports user defined vdd), processor modeling based on modern processors
- Flexible, extensible, and high (i.e., architecture) level framework
  - A framework for architecture research
  - Flexible to make researchers's life easier
    - Pre-populated micro-architecture configurations (can be changed by experienced users too!)
    - Multilevel automatic optimization 
  - Hierarchical modeling framework for easy extension and porting  
    - Standalone for TDP
    - Paired up with performance simulators (or machine profiling statistics) for fine-grained study

What McPAT is NOT:
- A hardware design EDA platform; nor a performance simulator
  - Use RTL/SPICE/...(not McPAT) if focusing on details of complex logic or analog components 
    - Empirical and curve-fitting based modeling for complex logic and analog building blocks (the most practical modeling methodology for high level framework). 
      - Solution1: Users replace those models with in-house models obtained from EDA tools
      - Solution2: Users contribute their EDA based detailed models back to the community for sharing
    - Use performance simulators for performance (McPAT cannot do performance simulations)
- A restrictive environment 
  - It is a framework (rather than a black-box tool)
    - Its built-in models are for references and for providing methodological examples. 
      - McPAT's built-in model includes simplified assumptions (e.g. unified instruction window for all instruction types)
      - McPAT provides building blocks so that it is composable 
      - Users should always understand the methodology when using the built-in models or compose their own models.				
- Finished!
  - There is always room for improvement . . .
  - Thanks for the continueous contributing from the user community!  

***

For complete documentation of the McPAT, please refer to the following paper,
*"McPAT: An Integrated Power, Area, and Timing Modeling
 Framework for Multicore and Manycore Architectures"*, 
that appears in MICRO 2009. Please cite the paper, if you use
McPAT in your work. The bibtex entry is provided below for your convenience.
```
@inproceedings{mcpat:micro,
  author = {Sheng Li and Jung Ho Ahn and Richard D. Strong and Jay B. Brockman and Dean M. Tullsen and Norman P. Jouppi},
  title =  "{McPAT: An Integrated Power, Area, and Timing Modeling Framework for Multicore and Manycore Architectures}",
  booktitle = {MICRO 42: Proceedings of the 42nd Annual IEEE/ACM International Symposium on Microarchitecture},
  year = {2009},
  pages = {469--480},
}
```

## Usage

McPAT takes input parameters from an XML-based interface,
then it computes area and peak power of the 
Please note that the peak power is the absolute worst case power, 
which could be even higher than TDP. 

### Build:
This fork of McPAT has 3 dependencies:
1. [CMake 3.12+](https://cmake.org/)
2. [LibBoost 1.56+](https://www.boost.org/)
3. [Clang](https://clang.llvm.org/)

To build McPAT:
1. Create a build directory:
    ```
    mkdir build
    ```
2. Change directory to the build directory:
    ```
    cd build/
    ```
3. Use CMake to Generate the Unix Makefiles. There are 3 build types:
  - Release - Enables -O3 optimizations
  - Profile - Enables -O3 optimazations and compiles with flags for profiling with [gprof](https://ftp.gnu.org/old-gnu/Manuals/gprof-2.9.1/html_mono/gprof.html)
  - Debug - Disables optimizations and compiles with debug flags for use with GDB
    ```
    cmake -DCMAKE_BUILD_TYPE=release ..
    ```
4. Make
    ```
    make clean && make -j
    ```

### Unit Testing:

This version of McPAT also comes with a unit testing framework for verifying your builds. It tests all the included processor description files and outputs from v1.3.0 and has support for unit testing the restoration from a serialized file.

To run the unit tests:
```
./unit_test.sh
```
This will run through all the test vectors and output the number of tests passed and test failed, along with providing a diff file for all the tests that failed. You can check the output directory for the diff file to see what was different from the expected output.


### Run:

To run McPAT outside of the unit testing framework, a configuration xml must be provided as an input.
```
./mcpat -i <file.xml>
```
To display the help message:
```
./mcpat --help
```
To create a serialization checkpoint:
```
./mcpat -i <infile.xml> --serial_create=true --serial_file=<path/to/serial/file.txt>
```
To restore from a serialization checkpoint:
```
./mcpat -i <infile.xml> --serial_restore=true --serial_file=<path/to/serial/file.txt>
```

## General Notes:

1. Optimization: McPAT will try its best to satisfy the target clock rate. When it cannot find a valid solution, it gives out warnings, while still giving a solution that is closest to the timing constraints and calculate power based on it. The optimization will lead to larger power/area numbers for target higher clockrate. McPAT also provides the option "-opt_for_clk" to turn on ("-opt_for_clk 1") and off this strict optimization for the timing constraint. When it is off, McPAT always optimize component for ED^2P without worrying about meeting the target clock frequency. By turning it off, the computation time can be reduced, which suites for situations where target clock rateis conservative.
2. Outputs: McPAT outputs results in a hierarchical manner. Increasing the "-print_level" will show detailed results inside each component. For each component, major parts are shown, and associated pipeline registers/control logic are added up in total area/power of each components. In general, McPAT does not model the area/overhead of the pad frame used in a processor die.
   
3. How to use the XML interface for McPAT 
    1. Set up the parameters Parameters of target designs need to be set in the *.xml file for entries tagged as "param". McPAT have very detailed parameter settings. Please remove the structure parameter from the file if you want to use the default values. Otherwise, the parameters in the xml file will override the default values. 
    2. Pass the statistics: There are two options to get the correct stats: 
        1. The performance simulator can capture all the stats in detail and pass them to McPAT;
        2. Performance simulator can only capture partial stats and pass them to McPAT, while McPAT can reason about the complete stats using the partial information and the configuration. Therefore, there are some overlap for the stats. 
   
4. Interface XML file structures (**PLEASE READ!**) The XML is hierarchical from processor level to micro-architecture level. McPAT support both heterogeneous and homogeneous manycore processors.
    1. For heterogeneous processor setup, each component (core, NoC, cache, and etc) must have its own instantiations (core0, core1, ..., coreN). Each instantiation will have different parameters as well as its stats. Thus, the XML file must have multiple "instantiation" of each type of heterogeneous components and the corresponding hetero flags must be set in the XML file. Then state in the XML should be the stats of "a" instantiation (e.g. "a" cores). The reported runtime dynamic is of a single instantiation (e.g. "a" cores). Since the stats for each (e.g. "a" cores) may be different, we will see a whole list of (e.g. "a" cores) with different dynamic power, and total power is just a sum of them.
    2. For homogeneous processors, the same method for heterogeneous can also be used by treating all homogeneous instantiations as heterogeneous. However, a preferred approach is to use a single representative for all the same components (e.g. core0 to represent all cores) and set the processor to have homogeneous components (e.g. <param name="homogeneous_cores" value="1"/> ). Thus, the XML file only has one instantiation to represent all others with the same architectural parameters. The corresponding homo flags must be set in the XML file.  Then, the stats in the XML should be the aggregated stats of the sum of all instantiations (e.g. aggregated stats of all cores). In the final results, McPAT will only report a single instantiation of each type of component, and the reported runtime dynamic poweris the sum of all instantiations of the same type. This approach can run fast and use much less memory.        

5. Guide for integrating McPAT into performance simulators and bypassing the XML interface The detailed work flow of McPAT has two phases: the initialization phase and the computation phase. Specifically, in order to start the initialization phase a user specifies static configurations, including parameters at all three levels, namely, architectural, circuit, and technology levels. During the initialization phase, McPAT will generate the internal chip representation using the configurations set by the user. The computation phase of McPAT is called by McPAT or the performance simulator during simulation to generate runtime power numbers. Before calling McPAT to compute runtime power numbers, the performance simulator needs to pass the statistics, namely, the activity factors of each individual components to McPAT via the XML interface. The initialization phase is very time-consuming, since it will repeat many times until valid configurations are found or the possible configurations are exhausted. To reduce the overhead, a user can let the simulator to call McPAT directly for computation phase and only call initialization phase once at the beginning of simulation. In this case, the XML interface file is bypassed, please refer to processor.cc to see how the two phases are called.
   
6. Sample input files: This package provide sample XML files for validating target processors. Please find the enclosed Niagara1.xml (for the Sun Niagara1 processor), Niagara2.xml (for the Sun Niagara2 processor), Alpha21364.xml (for the Alpha21364 processor), Xeon.xml (for the Intel Xeon Tulsa processor), and ARM_A9_2GHz.xml (for ARM Cortex A9 hard core 2GHz implementation from ARM) 
   
7. Modeling of power management techniques: McPAT supports both DVS and power-gating. For DVS, users can use default ITRS projected vdd at each technology node as supply voltage at DVS level 0 (DVS0) or define voltage at DVS0. For power-gating, McPAT supports both default power-saving virtual supply voltage computed automatically using technology parameters. Default means using technology (ITRS based) lowest value for state-retaining power-gating User can also defined voltage for Power-saving states, as shown in example file of Xeon.xml (search for power_gating_vcc). When using user-defined power-saving virtual supply voltage, please understand the implications when setting up voltage for different sleep states. For example, when deep sleep state is used (voltage lower than the technology allowed state retaining supply voltage), the effects of losing data and cold start effects (beyond the scope of McPAT) must be considered when waking up the architecture. Power-gating and DVS cannot happen at the same time. Because power-gating happens when circuit is idle, while DVS happens when circuit blocks are active. 
   
***

McPAT includes its special version of Cacti (called Cacti-P) based on Cacti6.5 release. The major changes of 
the special Cacti, called Cacti-P in this distro, (compared to cacti6.5) include the following new features. 
The inclosed Cacti-P can run stand-alone if users want to use these features.
 
 * CAM and fully associative cache modeling
 * Improved leakage power modeling with consideration of device/gate topology
 * long channel device for reduce sub-threshold leakage power 
 * Sleep transistor based power-gating modeling
 * gate leakage power
 * Support user defined voltage supply (Vdd)
 * Dynamic voltage scaling (DVS)
 
For complete documentation of Cacti-P, please refer to the following paper,
*"CACTI-P: Architecture-Level Modeling for SRAM-based Structures with Advanced Leakage Reduction Techniques"*, 
that appeared in ICCAD2011. Please cite the paper, if you use
Cacti-P in your work. The bibtex entry is provided below for your convenience.

```
@inproceedings{cacti-p:iccad,
  author = {Sheng Li and Ke Chen and Jung Ho Ahn and Jay B. Brockman and Norman P. Jouppi},
  title = {CACTI-P: Architecture-level modeling for SRAM-based structures with advanced leakage reduction techniques},
  booktitle = {ICCAD: International Conference on Computer-Aided Design},
  year = {2011},
  pages = {694-701},
}
```

 
***

McPAT uses an opensource XML parser written and kindly specially licensed by Mr. Frank Vanden Berghen. 
The detailed information about this XML parser can be found at the license information in xmlParse.cc/xmlParse.h       

***

McPAT is in its beginning stage. We are still improving the tool. 
Please come back to its website for newer versions.
McPAT has been constantly and rapidly improved with new models and latest technology. 
Please always refer to its code for most up-to-date and most accurate information. 
If you have any comments, questions, or suggestions, please write to us:


Sheng Li             
Sheng.sli@gmail.com 




