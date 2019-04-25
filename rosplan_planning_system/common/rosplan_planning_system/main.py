#!/usr/bin/env python

import argparse
import sys

from pysmt.shortcuts import get_env, Real
from pysmt.logics import LRA
from fractions import Fraction


from rosplan_planning_system.pyrobustenvelope import validate_stn_plan, synthesis_stn_plan
from rosplan_planning_system.pyrobustenvelope import compute_envelope, compute_envelope_construct
from rosplan_planning_system.pyrobustenvelope.stn import STN

def main():
    parser = argparse.ArgumentParser(description='Compute robust envelopes with SMT')
    parser.add_argument('command', type=str,
                        choices=['validate', 'synthesize', 'envelope', 'plot-plan', 'construct'])
    parser.add_argument('domain', type=str)
    parser.add_argument('problem', type=str)
    parser.add_argument('plan', type=str)

    parser.add_argument('--splitting', '-S', type=str,
                        choices=["monolithic", "partial", "full"],
                        default=None)
    parser.add_argument('--epsilon', '-E', type=str,
                        default="0.001")

    parser.add_argument('--qelim', '-q', type=str,
                        choices=list(get_env().factory.all_quantifier_eliminators(logic=LRA)),
                        default=None)
    parser.add_argument('--solver', '-s', type=str,
                        choices=list(get_env().factory.all_solvers()),
                        default=None)
    parser.add_argument('--early-elimination', '-e', action='store_true',
                        default=False, help='Eliminate forall quantifiers asap '
             '(allows to use a solver that only supports QF_LRA in validation)')
    parser.add_argument('--no-compact-encoding', '-c', action='store_true',
                        default=False, help='Disable compactness analysis')
    parser.add_argument('--no-simplify-effects', '-n', action='store_true',
                        default=False, help='Disable compactness analysis')
    parser.add_argument('--no-learning', '-l', action='store_true',
                        default=False, help='Assert learned proof obligations '
                        'when performing validation')
    parser.add_argument('--assume-positive', '-A', action='store_true',
                        default=False, help='Assumes all parameters are positive')

    parser.add_argument('--bound', '-b', type=float,
                        default=1, help='Bound for construct algorithm')

    parser.add_argument('--timeout', '-t', type=int,
                        default=60, help='timeout for construct algorithm')


    parser.add_argument('--debug', '-d', action='store_true',
                        default=False, help='Print debug information')

    parser.add_argument('--plot-region', '-p', action='store_true',
                        default=False, help='Plot synthesized region')
    parser.add_argument('--plot-min', '-m', type=float,
                        default=0, help='Minimum value for plot sampling')
    parser.add_argument('--plot-max', '-M', type=float,
                        default=200, help='maximum value for plot sampling')
    parser.add_argument('--plot-step', '-P', type=float,
                        default=1, help='sampling step value for plotting')
    parser.add_argument('--plot-data', '-D', action='store_true',
                        default=False, help='print plot data')

    parser.add_argument('--output', '-o', type=str)

    args = parser.parse_args()

    if args.output:
        sys.stdout = open(args.output, 'w')

    # Empirically, Mathsat qe seems to work better than Z3
    get_env().factory.set_qelim_preference_list(['msat_fm', 'msat_lw', 'z3',
                                                 'bdd', 'shannon'])

    compact_encoding = not args.no_compact_encoding
    simplify_effects = not args.no_simplify_effects
    if args.command == 'validate':
        result, debug = validate_stn_plan(args.domain, args.problem, args.plan,
                                          early_forall_elimination=args.early_elimination,
                                          compact_encoding=compact_encoding,
                                          debug=args.debug, solver=args.solver,
                                          learn=not args.no_learning,
                                          splitting=args.splitting,
                                          epsilon=Fraction(args.epsilon),
                                          simplify_effects=simplify_effects)
        if result:
            print('The plan is valid for the given problem instance')
        else:
            print('The plan is *NOT* valid for the given problem instance')
            print('The reported error is:')
            print(debug)
    elif args.command == 'plot-plan':
        stn = STN(args.plan)
        print(stn.to_dot())

    elif args.command == 'synthesize':
        res, params = synthesis_stn_plan(args.domain, args.problem, args.plan,
                                         debug=args.debug, splitting=args.splitting,
                                         early_forall_elimination=args.early_elimination,
                                         compact_encoding=compact_encoding,
                                         solver=args.solver,
                                         qelim_name=args.qelim,
                                         epsilon=Fraction(args.epsilon),
                                         simplify_effects=simplify_effects,
                                         learn=not args.no_learning)
        print(res.serialize())
        if args.plot_region:
            if len(params) != 2:
                raise ValueError("Unable to plot more than 2 parameters (%d in this case)" % len(params))
            import matplotlib.pyplot as plt
            import numpy as np
            x = []
            y = []
            pars = list(params[n] for n in sorted(params, key=lambda x: x.name))
            names = list(p.name for p in sorted(params, key=lambda x: x.name))
            for vx in np.arange(args.plot_min, args.plot_max, args.plot_step):
                part = res.substitute({pars[0] : Real(float(vx))}).simplify()
                if not part.is_false():
                    for vy in np.arange(args.plot_min, args.plot_max, args.plot_step):
                        check = part.substitute({pars[1] : Real(float(vy))}).simplify()
                        if check.is_true():
                            x.append(vx)
                            y.append(vy)
            if args.plot_data:
                print("Plot = [")
                for t in zip(x, y):
                    print("%s, %s" % t)
                print("]")
            else:
                plt.scatter(x, y)
                plt.axis([args.plot_min, args.plot_max, args.plot_min, args.plot_max])
                ax = plt.gca()
                ax.set_autoscale_on(False)
                plt.xlabel(names[0])
                plt.ylabel(names[1])
                plt.show()


    elif args.command == 'envelope':
        res = compute_envelope(args.domain, args.problem, args.plan,
                               debug=args.debug, splitting=args.splitting,
                               early_forall_elimination=args.early_elimination,
                               compact_encoding=compact_encoding,
                               solver=args.solver,
                               qelim_name=args.qelim,
                               epsilon=Fraction(args.epsilon),
                               simplify_effects=simplify_effects,
                               learn=not args.no_learning)
        if res:
            for p, (l, u) in res.items():
                print("%s in [%s, %s]" % (p.name, float(l.constant_value()), float(u.constant_value())))
        else:
            print("The problem is unsatisfiable!")

    elif args.command == 'construct':
        stream = sys.stdout
        if args.output:
            stream = open(args.output, 'w')

        debug = None
        if args.debug:
            debug = stream

        res = compute_envelope_construct(args.domain, args.problem, args.plan,
                               debug=debug, splitting=args.splitting,
                               early_forall_elimination=args.early_elimination,
                               compact_encoding=compact_encoding,
                               solver=args.solver,
                               qelim_name=args.qelim,
                               epsilon=Fraction(args.epsilon),
                               simplify_effects=simplify_effects,
                               bound=args.bound,
                               assume_positive=args.assume_positive,
                               timeout=args.timeout)
        if res:
            for p, (l, u) in res.items():
                stream.write("%s in [%s, %s]\n" % (p.name, l, u))
        else:
            stream.write("The problem is unsatisfiable!\n")

        if args.output:
            stream.close()

    else:
        print("ERROR: unknown command given '%s'" % args.command)


    if args.output and args.command != 'construct':
        sys.stdout.close()

if __name__ == "__main__":
    main()

