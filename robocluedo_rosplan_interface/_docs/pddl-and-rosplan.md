# PDDL and ROSPlan

## validation wit Parser

from inside the folder `(package robocluedo_rosplan_interface)/pddl/` run the command

```bash
./parser <domain file> <(opt) problem file>
```

## Specific rules

- seemingly there's no bugs in tabulation parsing

- **always specify the type of the objects and the parameters!** Typing is a requirement in ROSPlan, nevertheless it is optional in the standard model 
	
	In particular ROSPlan can't deal with the *pure classical planning*: it works fine only with `durative-actions`. 
	
- durative actions with empty parameters are *allowed* by the system

- *don't use `(and ...)` with only one argument!* If the and is going to contain only one predicate, simply don't use such operators: just write the predicate, and nothing else. 

- **empty `:condition (... )` statements are not allowed!**

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

When you set the requirement *ADL*, the message is this:

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

## Metric-ff help

```

```

## Contingent-FF help

```

```

## PANDA help

```

```
