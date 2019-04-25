import time
from itertools import cycle

from rosplan_planning_system.pypddlplus import parse_instance
from pysmt.shortcuts import (Implies, And, Not, qelim, Solver, ForAll, GE, LE,
                             FreshSymbol, Plus, Minus, Exists, Times,
                             Real, is_valid, qelim, get_model, is_sat)
import pysmt.typing as types
from pysmt.logics import QF_LRA, LRA
from pysmt.oracles import get_logic

from rosplan_planning_system.pyrobustenvelope.stn import STN
from rosplan_planning_system.pyrobustenvelope.encoder import Encoder
from rosplan_planning_system.pyrobustenvelope.construct import ConstructAlgorithm


import ctypes

def terminate_thread(thread):
    """Terminates a python thread from another thread.

    :param thread: a threading.Thread instance
    """
    if not thread.is_alive():
        return

    exc = ctypes.py_object(SystemExit)
    res = ctypes.pythonapi.PyThreadState_SetAsyncExc(
        ctypes.c_long(thread.ident), exc)
    if res == 0:
        raise ValueError("nonexistent thread id")
    elif res > 1:
        # """if it returns a number greater than one, you're in trouble,
        # and you should call it again with exc=NULL to revert the effect"""
        ctypes.pythonapi.PyThreadState_SetAsyncExc(thread.ident, None)
        raise SystemError("PyThreadState_SetAsyncExc failed")



def validate_stn_plan(domain_fname, problem_fname, plan_fname, debug=False,
                      early_forall_elimination=False, compact_encoding=True,
                      learn=False, solver=None, splitting=None, epsilon=None,
                      simplify_effects=True):
    if splitting is None:
        splitting = "full"
    assert splitting in ["monolithic", "partial", "full"]
    stn = STN(plan_fname)
    if stn.is_parametric():
        raise ValueError("Cannot validate a parametric plan!")

    instance = parse_instance(domain_fname, problem_fname)

    enc = Encoder(instance, stn, debug=debug,
                  early_elimination=early_forall_elimination,
                  compact_encoding=compact_encoding,
                  solver=solver, epsilon=epsilon,
                  simplify_effects=simplify_effects,
                  learning=learn)

    l = LRA
    if early_forall_elimination:
        l = QF_LRA

    assumptions, proof_obligations = enc.encode()
    if len(enc.parameter_vars) != 0:
        raise ValueError("Unable to validate a parametric problem instance and plan!")

    with Solver(name=solver, logic=l) as s:
        s.add_assertion(And(assumptions))

        exist_check = s.solve()
        if not exist_check:
            return False, "Plan is unrealizable"

        if splitting == 'monolithic':
            constraints = And(y for x in proof_obligations for y in x[1])
            s.add_assertion(Not(constraints))
            valid_check = s.solve()
            if valid_check:
                return False, "Plan Invalid"
            return True, None
        else:
            for k, fs in proof_obligations:
                if debug:
                    print("Checking %s..." % k)
                split = fs
                if splitting == 'partial':
                    split = [And(fs)]
                for f in split:
                    res = s.solve([Not(f)])
                    if not res:
                        # UNSAT
                        if debug:
                            print(" - Ok")
                        if learn:
                            s.add_assertion(f) # Learn f
                    else:
                        if debug:
                            print(" - Failed! %s" % f.serialize())
                            m = s.get_model()
                            res = []
                            for var, v in m:
                                res.append("%s = %s" % (var, v))
                            res.sort()
                            print("\n".join(res))
                        return False, k + " invalid"
            return True, None



def synthesis_stn_plan(domain_fname, problem_fname, plan_fname, debug=False,
                       splitting=None, early_forall_elimination=False,
                       compact_encoding=True, solver=None, qelim_name=None,
                       epsilon=None, simplify_effects=True, learn=False):
    if splitting is None:
        splitting = "partial"
    assert splitting in ["monolithic", "partial", "full"]

    stn = STN(plan_fname)

    instance = parse_instance(domain_fname, problem_fname)
    enc = Encoder(instance, stn, debug=debug,
                  early_elimination=early_forall_elimination,
                  compact_encoding=compact_encoding,
                  solver=solver, qelim_name=qelim_name, epsilon=epsilon,
                  simplify_effects=simplify_effects, learning=learn)

    f = enc.synthesize_parameter_region(splitting=splitting)
    return f.simplify(), enc.parameter_vars


def compute_envelope(domain_fname, problem_fname, plan_fname, debug=False,
                     splitting=None, early_forall_elimination=False,
                     compact_encoding=True, solver=None, qelim_name=None,
                     epsilon=None, simplify_effects=True, learn=False):
    from pysmt.shortcuts import Optimizer

    if splitting is None:
        splitting = "partial"

    if debug: print("Computing region...")
    region, params = synthesis_stn_plan(domain_fname, problem_fname, plan_fname,
                            debug=debug,
                            early_forall_elimination=early_forall_elimination,
                            compact_encoding=compact_encoding,
                            splitting=splitting,
                            solver=solver,
                            qelim_name=qelim_name, epsilon=epsilon,
                            simplify_effects=simplify_effects,
                            learn=learn)
    if debug: print("done.")

    if debug: print("Computing region of bounds...")
    implicant = []
    bounds = {}
    for p, pv in params.items():
        lb = FreshSymbol(types.REAL, template="lb_%s_%%d" % p.name)
        ub = FreshSymbol(types.REAL, template="ub_%s_%%d" % p.name)

        bounds[p] = (lb, ub)

        implicant.append(And(GE(pv, lb), LE(pv, ub)))

    f = ForAll(set(params.values()), Implies(And(implicant), region))
    qe_f = qelim(f, solver_name=qelim_name)
    bounds_region = And(qe_f, And(LE(lb, ub) for (lb, ub) in bounds.values()))
    if debug: print("done.")

    if debug: print("Finding maximal bounds...")
    tomin = Plus(Times(Real(p.weight), Minus(l, u)) for p, (l, u) in bounds.items())

    opt = Optimizer(name='z3', logic=QF_LRA)
    opt.add_assertion(bounds_region)

    loops = [30]
    def ensure_quit(model):
        if debug: print("OPT: %s" % model.get_value(Times(Real(-1), tomin)))
        loops[0] -= 1
        return loops[0] <= 0

    optimal = opt.optimize(tomin, callback=ensure_quit)

    if optimal is None:
        return None
    else:
        envelope = {}
        for p in params:
            lb, ub = bounds[p]
            lv = optimal.get_value(lb)
            uv = optimal.get_value(ub)
            envelope[p] = (lv, uv)
        return envelope

    # max_value = None
    # max_loops = 50
    # envelope = None
    # with Solver(name=solver, logic=QF_LRA) as s:
    #     s.add_assertion(bounds_region)
    #     while max_loops > 0:
    #         max_loops -= 1
    #         if max_value is not None:
    #             s.add_assertion(GT(tomax, max_value))
    #         r = s.solve()
    #         if r:
    #             max_value = s.get_value(tomax)
    #             envelope = {}
    #             for p in params:
    #                 lb, ub = bounds[p]
    #                 lv = s.get_value(lb)
    #                 uv = s.get_value(ub)
    #                 envelope[p] = (lv, uv)
    #         else:
    #             break
    #         if debug: print("Found an envelope with optimality value: %s" % max_value)
    # if debug: print("done.")

    # return envelope






def compute_envelope_construct(domain_fname, problem_fname, plan_fname, debug=False,
                               splitting=None, early_forall_elimination=False,
                               compact_encoding=True, solver=None, qelim_name=None,
                               epsilon=None, simplify_effects=True, rectangle_callback=None,
                               bound=1, assume_positive=False, timeout=None):
    """This function iteratively constructs a rectangular robustness envelope

    :param domain_fname: The path containing the domain PDDL file
    :type domain_fname: str

    :param problem_fname: The path containing the problem PDDL file
    :type problem_fname: str

    :param plan_fname: The path containing the parametrized STN plan file
    :type plan_fname: str

    :param debug: (optional) If not none, specifies the file-like
    stream where to write the program execution log. By default,
    nothing is printed.
    :type debug: None or file-like object

    :param splitting: (optional) Can be either 'full', 'monolithic' or
    'partial' and specifies how and if to de-compose the universal
    check among different solvers. Default is 'monolithic'
    :type splitting: str or None

    :param early_forall_elimination: (optional) If set to true, the
    encoding is pre-processed with a quantifier-elimination procedure
    to get rid of quantifiers. This needs to be true if the solver
    specified with `solver` parameter dies not support quantified
    formulae. Default is False.
    :type early_forall_elimination: bool

    :param compact_encoding: (optional) If set to `True`, the encoding
    is simplified by pre-computing the possible position of each
    happening allowed by the given STN plan. Default is True.
    :type compact_encoding: bool

    :param solver: (optional) the name of the solver to use for
    SMT-checks, can be any solver supported by pysmt and installed on
    the system. By default, we let pysmt decide the best solver.
    :type solver: str

    :param qelim_name: (optional) the name of the quantifier
    eliminator to use, can be any QE supported by pysmt and installed
    on the system. By default, we let pysmt decide the best QE.
    :type qelim_name: str

    :param epsilon: (optional) the epsilon-separation to use in the encoding
    expressed in time-units. If None is specified the encoder assumes
    0.001 (default).
    :type epsilon: None or float

    :param simplify_effects: (optional) Construct the encoding to summarize
    effects at the same time. Default is True.
    :type simplify_effects: bool

    :param rectangle_callback: (optional) A function to be called each time a new
    rectangle is found by the algorithm. Since the approach is
    anytime, all the rectangles passed to this functions are
    incrementally-better under-approximations of the algorithm result.
    A rectangle is represented as a dictionary mapping parameters to
    pairs of floats. E.g. {Parameter('a') : (10, 20)} indicates that
    parameter a is between 10 and 20. By default, no function is called.
    :type rectangle_callback: a function taking in input a rectangle

    :param bound: The convergence value. If x is specified, the
    rectangle generated will have bounds that are at most x distant
    from the border of the real envelope. (default is 1)

    :returns: the final rectangle. A rectangle is represented as a
    dictionary mapping parameters to pairs of
    floats. E.g. {Parameter('a') : (10, 20)} indicates that parameter
    a is between 10 and 20

    """

    if splitting is None:
        splitting = "monolithic"
    assert splitting in ["monolithic", "partial", "full"]

    stn = STN(plan_fname)
    instance = parse_instance(domain_fname, problem_fname)
    enc = Encoder(instance, stn, debug=debug,
                  early_elimination=early_forall_elimination,
                  compact_encoding=compact_encoding,
                  solver=solver, qelim_name=qelim_name, epsilon=epsilon,
                  simplify_effects=simplify_effects,
                  assume_positive_params=assume_positive)

    c = ConstructAlgorithm(instance, stn, enc, debug=debug, splitting=splitting,
                           solver=solver, qelim_name=qelim_name, bound=bound)

    if timeout is None:
        return c.run(rectangle_callback=rectangle_callback)
    else:
        import threading

        rects = []
        def cb(r):
            rects.append(r)
            if rectangle_callback:
                rectangle_callback(r)

        p = threading.Thread(target=c.run, args=(cb, ))
        p.start()

        # Wait for timeout seconds or until process finishes
        p.join(timeout)

        # If thread is still active
        if p.is_alive():
            terminate_thread(p)

        if len(rects):
            return {p:v for (p, _), v in rects[-1].items()}
        else:
            return None
