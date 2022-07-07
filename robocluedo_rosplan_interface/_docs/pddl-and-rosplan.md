# PDDL and ROSPlan

## validation wit Parser

from inside the folder `(package robocluedo_rosplan_interface)/pddl/` run the command

```bash
./parser <domain file> <(opt) problem file>
```

## Specific rules for the PDDL in ROS Plan

you can set the planner in a suitable way for your application by the `planner_command` parameter in the planner interface. BUT the knowledge base node is able to interpret a non-standard particular version of PDDL, otherwise it closes unexpectedly. 

- use this list of requirements:

	```lisp
	(:requirements 
		:strips 
		:typing 
		:equality 
		:universal-preconditions 
		:durative-actions
	)
	```

- seemingly there's no bugs in tabulation parsing

- **always specify the type of the objects and the parameters!** Typing is a requirement in ROSPlan, nevertheless it is optional in the standard model 
	
	In particular ROSPlan can't deal with the *pure classical planning*: it works fine only with `durative-actions`. 
	
- durative actions with empty parameters are *allowed* by the system

- *don't use `(and ...)` with only one argument!* If the and is going to contain only one predicate, simply don't use such operators: just write the predicate, and nothing else. 

- **empty `:condition (... )` statements are not allowed!**

- you cannot use `(not ...)` inside the goal statement: the planner doesn't fully support `:adl` requirement. 

## Durative Actions - the right syntax

```lisp
(:durative-action ???
	:parameters ( ??? )
	
	:duration (= ?duration 1)
	
	:condition ( ??? )
	
	:effect (and
		???
	)
)
```

## ADL support

**ROSPlan doesn't have a complete support for ADL**, which is definitely a bad news. Classical planning isn't supported, in particular the common actions. When you set the requirement *ADL*, the message is this:

```
A problem has been encountered, and the planner has to terminate.
-----------------------------------------------------------------
Unfortunately, at present, the planner does not fully support ADL
unless in the rules for derived predicates.  Only two aspects of
ADL can be used in action definitions:
- forall conditions, containing a simple conjunct of propositional and
  numeric facts;
- Conditional (when... ) effects, and then only with numeric conditions
  and numeric consequences on values which do not appear in the
  preconditions of actions.

To use this planner with your problem, you will have to reformulate it to
avoid ADL.  Alternatively, if you have a particularly compelling case
for them, please contact the authors to discuss it with them, who may be able to
extend the planner to meet your needs.
```

## Available Planners

**Attention** : Better to use POPF, as suggested in the official tutorial. 

## POPF help

see [popf infos](https://planning.wiki/ref/planners/popf)

**temporal planner (supporting durative actions only)**. Supported requirements:

```lisp
(:requirements 
	:strips 
	:typing 
	:equality 
	:universal-preconditions 
	:durative-actions
)
```

**PLanning interface** : `<node pkg="rosplan_planning_system" name="popf_planning_interface" type="popf_planning_interface">`

help from the program:

```
root@dbfbde77a543:~/ros_ws/src/ROSPlan/rosplan_planning_system/common/bin# ./popf --help
Unrecognised command-line switch '-'
POPF, Release 2
By releasing this code we imply no warranty as to its reliability
and its use is entirely at your own risk.

Usage: ./popf [OPTIONS] domainfile problemfile [planfile, if -r specified]

Options are: 

	-citation	Display citation to relevant conference paper (ICAPS 2010);
	-b		Disable best-first search - if EHC fails, abort;
	-E		Skip EHC: go straight to best-first search;
	-e		Use standard EHC instead of steepest descent;
	-h		Disable helpful-action pruning;
	-k		Disable compression-safe action detection;
	-c		Disable the tie-breaking in RPG that favour actions that slot into the partial order earlier;
	-S		Sort initial layer facts in RPG by availability order (only use if using -c);
	-m		Disable the tie-breaking in search that favours plans with shorter makespans;
	-F		Full FF helpful actions (rather than just those in the RP applicable in the current state);
	-r		Read in a plan instead of planning;
	-T		Rather than building a partial order, build a total-order
	-n		Optimise, finding many solutions;
	-v<n>		Verbose to degree n (n defaults to 1 if not specified).
	-L<n>		LP verbose to degree n (n defaults to 1 if not specified).
```

**Command line for POPF** : for normal planning,

```bash
timeout 10 <popf_path>/popf -v2 DOMAIN PROBLEM > plan.pddl
```

*optimal planning if possible* : (it could significantly slow down the planning workflow)

```bash
<popf_path>/popf -n -v2 DOMAIN PROBLEM > plan.pddl
```

## Metric-ff help

see [Metric-FF on planning.wiki](https://planning.wiki/ref/planners/metricff)

see also the [official page of Metric-FF](https://fai.cs.uni-saarland.de/hoffmann/metric-ff.html)

**temporal and numerical planning (with support for ADL)**. Supported requirements:

```
(:requirements 
	;; === PDDL1.1 === ;;
	:strips 
	:typing 
	:equality 
	:existential-preconditions
	:universal-preconditions 
	:conditional-effects
	:quantified-preconditions
	
	;; === PDDL2.1 === ;;
	:fluents
	:durative-actions
)
```

help from metric-ff:

```
root@dbfbde77a543:~/ros_ws/src/ROSPlan/rosplan_planning_system/common/bin# ./Metric-FF --help

usage of ff:

OPTIONS   DESCRIPTIONS

-p <str>    Path for operator and fact file
-o <str>    Operator file name
-f <str>    Fact file name

-r <int>    Random seed [used for random restarts; preset: 0]

-s <int>    Search configuration [preset: s=5]; '+H': helpful actions pruning
      0     Standard-FF: EHC+H then BFS (cost minimization: NO)
      1     BFS (cost minimization: NO)
      2     BFS+H (cost minimization: NO)
      3     Weighted A* (cost minimization: YES)
      4     A*epsilon (cost minimization: YES)
      5     EHC+H then A*epsilon (cost minimization: YES)
-w <num>    Set weight w for search configs 3,4,5 [preset: w=5]

-C          Do NOT use cost-minimizing relaxed plans for options 3,4,5

-b <float>  Fixed upper bound on solution cost (prune based on g+hmax); active only with cost minimization

```

**Command line for Metric-FF** : 

```bash
timeout 10 <metric-ff_path>/Metric-FF -o DOMAIN -p PROBLEM 
```

