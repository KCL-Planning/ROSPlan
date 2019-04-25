import time

from fractions import Fraction

import pysmt.typing as types

import rosplan_planning_system.pypddlplus.model as pddl
from pysmt.shortcuts import get_env, Solver, qelim, is_sat, get_model

from rosplan_planning_system.pyrobustenvelope.stn import Parameter, TimePoint, Constraint

class Encoder(object):

    def __init__(self, instance, stn, env=None,
                 debug=False,
                 early_elimination=False,
                 compact_encoding=True,
                 solver=None, qelim_name=None,
                 epsilon=None, simplify_effects=False,
                 learning=True,
                 assume_positive_params=False):
        self.env = env if env else get_env()
        self.mgr = self.env.formula_manager
        self.instance = instance
        self.ground_instance = instance.ground()
        self.stn = stn
        self.debug = debug
        self.early_elimination = early_elimination
        self.compact_encoding = compact_encoding
        self.learning = learning
        self.assume_positive_params = assume_positive_params

        # Handle TILs by adding them to the stn
        init = self.ground_instance.problem.initial_state
        til_cnt = 0
        tils_cache = {}
        for x in init:
            if isinstance(x, pddl.PddlAtTime):
                if x.time in tils_cache: # Already found a TIL at the same time!
                    tp = tils_cache[x.time]
                    tp.til = tuple(list(tp.til) + [x.expression])
                else:
                    til_name = "til_%d" % til_cnt
                    til_cnt += 1
                    tp = TimePoint(til_name, kind="til", til=(x.expression,))
                    self.stn.time_points[til_name] = tp
                    self.stn.constraints.append(Constraint(self.stn.zero, tp, x.time, x.time))
                    tils_cache[x.time] = tp

        if epsilon is None:
            self.epsilon = Fraction("0.001")
        else:
            self.epsilon = epsilon

        self.solver_name = solver
        self.qelim_name = qelim_name

        if instance.domain.events:
            raise NotImplementedError("Events are not supported yet!")
        if instance.domain.processes:
            raise NotImplementedError("Processes are not supported yet!")

        self.fluent_vars = {}
        self.time_point_vars = {}
        self.time_point_pos_vars = {}
        self.time_vars = {}
        self.parameter_vars = {}

        self.tps_z = [self.stn.time_points[n] for n in sorted(self.stn.time_points)
                      if self.stn.time_points[n] != self.stn.zero]
        self.all_tps = [self.stn.zero] + self.tps_z
        self.H = len(self.stn.time_points)

        if not self.debug:
            self.t_var = self.mgr.FreshSymbol(types.REAL, template="__time__%d")
        else:
            self.t_var = self.mgr.Symbol("t", types.REAL)

        self.action_instances = []
        for tp in self.tps_z:
            if tp.is_start():
                action = self.instance.domain.durative_actions[tp.action_name]
                par_obj = [self.instance.problem.objects[x] for x in tp.parameters]
                g_name = tp.action_name
                if par_obj:
                    g_name = action.name + "_" + "_".join(x.name for x in par_obj)
                ground_action = self.ground_instance.domain.durative_actions[g_name]
                x = (ground_action, (tp, tp.paired_time_point))
                self.action_instances.append(x)
            elif tp.is_instantaneous_action():
                action = self.instance.domain.actions[tp.action_name]
                par_obj = [self.instance.problem.objects[x] for x in tp.parameters]
                g_name = tp.action_name
                if par_obj:
                    g_name = action.name + "_" + "_".join(x.name for x in par_obj)
                ground_action = self.ground_instance.domain.actions[g_name]
                x = (ground_action, (tp,))
                self.action_instances.append(x)

        self.possible_positions = {}
        self.possible_positions[self.stn.zero] = [0]
        if self.compact_encoding:
            self.compute_possible_positions()
        else:
            for tp in self.tps_z:
                self.possible_positions[tp] = list(range(1, self.H))
        for tp,v in self.possible_positions.items():
            print('%s : %s' % (tp.name, v))

        self.affected_variables = None
        if simplify_effects:
            self.affected_variables = {k: set() for k in range(1, self.H)}
            for tp in self.tps_z:
                for k in self.possible_positions[tp]:
                    literals = self.get_effect_literals(tp)
                    affected = set()
                    for l in literals:
                        if isinstance(l, pddl.PddlIncrease):
                            fun = l.function.function
                            affected.add(fun)
                        elif isinstance(l, pddl.PddlDecrease):
                            fun = l.function.function
                            affected.add(fun)
                        elif isinstance(l, pddl.PddlAssign):
                            fun = l.function.function
                            affected.add(fun)
                        elif isinstance(l, pddl.PddlNot):
                            pred = l.expression.function
                            affected.add(pred)
                        elif isinstance(l, pddl.PddlApply):
                            affected.add(l.function)
                        else:
                            raise NotImplementedError("Unknown effect kind: %s" % l)
                    self.affected_variables[k] |= affected

    def compute_possible_positions(self):
        with Solver(name=self.solver_name, logic="QF_LRA") as s:
            s.add_assertion(self.mgr.And(self.encode_imp()))

            for tp in self.tps_z:
                sat = False
                self.possible_positions[tp] = set()
                tpv = self.time_point(tp)
                s.push()
                lts = self.mgr.Plus(self.mgr.Ite(self.mgr.LT(self.time_point(oth), tpv),
                                                 self.mgr.Real(1), self.mgr.Real(0))
                                    for oth in self.tps_z)
                gts = self.mgr.Plus(self.mgr.Ite(self.mgr.GT(self.time_point(oth), tpv),
                                                 self.mgr.Real(1), self.mgr.Real(0))
                                    for oth in self.tps_z)
                ltv = self.mgr.FreshSymbol(types.REAL)
                gtv = self.mgr.FreshSymbol(types.REAL)
                s.add_assertion(self.mgr.Equals(ltv, lts))
                s.add_assertion(self.mgr.Equals(gtv, gts))
                while s.solve():
                    sat = True


                    lv = s.get_value(ltv)
                    gv = s.get_value(gtv)
                    l = int(lv.constant_value())
                    u = int(gv.constant_value())


                    for x in range(l, self.H-u-1):
                        self.possible_positions[tp].add(x+1)

                    # print("=" * 40)
                    # print("[%s, %s]" % (l, self.H-u-2))
                    # for tp,v in self.possible_positions.items():
                    #     print('%s : %s' % (tp.name, v))

                    start = None
                    end = None
                    for x in sorted(self.possible_positions[tp]):
                        if start is None:
                            start = x - 1
                            end = x - 1
                        elif x-1 == end+1:
                            end = x - 1
                        else:
                            lb = self.mgr.Real(start)
                            gb = self.mgr.Real(self.H - end - 2)
                            s.add_assertion(self.mgr.Or(self.mgr.LT(gtv, gb),
                                                 self.mgr.LT(ltv, lb)))
                            start = x - 1
                            end = x - 1
                    lb = self.mgr.Real(start)
                    gb = self.mgr.Real(self.H - end - 2)
                    s.add_assertion(self.mgr.Or(self.mgr.LT(gtv, gb),
                                         self.mgr.LT(ltv, lb)))

                s.pop()

                if not sat:
                    raise ValueError("The input plan is trivially unsatisfiable,"
                                     " time point %s is not schedulable!" % tp.name)

    def compute_possible_positions_fact(self):
        with Solver(name=self.solver_name, logic="QF_LRA") as s:
            s.add_assertion(self.mgr.And(self.encode_imp()))

            for tp in self.tps_z:
                self.possible_positions[tp] = set()

            sat = False
            o = list(self.tps_z)
            while s.solve():
                sat = True
                o.sort(key=lambda x : s.get_py_value(self.time_point(x)))

                prevs = []
                pv = None
                pidx = -1
                for i,tp in enumerate(o):
                    v = s.get_py_value(self.time_point(tp))
                    prevs.append(tp)
                    if v != pv:
                        prevs = [tp]
                        pidx = i
                        pv = v
                    for p in prevs:
                        for x in range(pidx, i+1):
                            self.possible_positions[p].add(x+1)

                f = []
                prev = o[0]
                pv = s.get_py_value(self.time_point(prev))
                for tp in o[1:]:
                    v = s.get_py_value(self.time_point(tp))
                    if v == pv:
                        f.append(self.mgr.GT(self.time_point(prev), self.time_point(tp)))
                    else:
                        f.append(self.mgr.GE(self.time_point(prev), self.time_point(tp)))
                    prev = tp
                    pv = v
                s.add_assertion(self.mgr.Or(f))


            if not sat:
                raise ValueError("The input plan is trivially unsatisfiable,"
                                 " time point %s is not schedulable!" % tp.name)

    def compute_possible_positions_old(self):
        with Solver(name=self.solver_name, logic="QF_LRA") as s:
            for tp in self.tps_z:
                s.add_assertion(self.mgr.GE(self.time_point(tp), self.mgr.Real(0)))
                pv = self.time_point_pos(tp)
                s.add_assertion(self.mgr.Or(self.mgr.Equals(pv, self.mgr.Real(x))
                                            for x in range(1, self.H)))

                for oth in self.tps_z:
                    if oth != tp:
                        s.add_assertion(self.mgr.Implies(self.mgr.GT(self.time_point(tp),
                                                                     self.time_point(oth)),
                          self.mgr.GT(self.time_point_pos(tp), self.time_point_pos(oth))))

            s.add_assertion(self.mgr.AllDifferent(self.time_point_pos(tp)
                                                  for tp in self.tps_z))

            for c in self.stn.constraints:
                sv = self.time_point(c.src)
                dv = self.time_point(c.dst)
                diff = self.mgr.Minus(dv, sv)
                if c.lower_bound is not None:
                    if isinstance(c.lower_bound, Parameter):
                        s.add_assertion(self.mgr.GE(diff, self.parameter(c.lower_bound)))
                    else:
                        s.add_assertion(self.mgr.GE(diff, self.mgr.Real(c.lower_bound)))
                if c.upper_bound is not None:
                    if isinstance(c.upper_bound, Parameter):
                        s.add_assertion(self.mgr.LE(diff, self.parameter(c.upper_bound)))
                    else:
                        s.add_assertion(self.mgr.LE(diff, self.mgr.Real(c.upper_bound)))

            for tp in self.tps_z:
                self.possible_positions[tp] = set()

            sat = False
            while s.solve():
                sat = True
                for tp in self.tps_z:
                    pv = self.time_point_pos(tp)
                    p = s.get_value(pv)
                    self.possible_positions[tp].add(int(p.constant_value()))
                # at least one possible value should be updated
                s.add_assertion(self.mgr.Or(self.mgr.And(self.mgr.Not(\
                    self.mgr.Equals(self.time_point_pos(tp), self.mgr.Real(v)))
                     for v in self.possible_positions[tp]) for tp in self.tps_z))

            if not sat:
                raise ValueError("The input plan is trivially unsatisfiable,")


    def float_expr(self, e, k, action=None, cond=False):
        if isinstance(e, pddl.PddlFloatConstant):
            return self.mgr.Real(e.value)
        elif isinstance(e, pddl.PddlDurationVar):
            assert action, "Duration variable used, but no action specified"
            s, d = action
            sv = self.time_point(s)
            dv = self.time_point(d)
            return self.mgr.Minus(dv, sv)
        elif isinstance(e, pddl.PddlSynthParam):
            if e in self.instance.domain.synth_parameters:
                return self.parameter(Parameter(e.name))
            else:
                s, _ = action
                return self.parameter(Parameter(s.instance_name + "." + e.name))

        elif isinstance(e, pddl.PddlApply):
            assert not e.actual_parameters
            if cond:
                # A continuous variable could be increased or decreased
                # continuously, so when reading the value at time k, we
                # need to consider this
                if k >= self.H:
                    # this is the final value
                    return self.fluent(e.function, self.H)
                else:
                    # This is an intermediate value, and we need to compensate
                    return self.mgr.Plus(self.fluent(e.function, k),
                                         self.encode_increment(e.function, k+1))
            else:
                return self.fluent(e.function, k)
        elif isinstance(e, pddl.PddlArithOperator):
            if e.op == "+":
                return self.mgr.Plus(self.float_expr(x, k, cond=cond, action=action) for x in e.operands)
            elif e.op == "*":
                return self.mgr.Times(self.float_expr(x, k, cond=cond, action=action) for x in e.operands)
            elif e.op == "-":
                assert len(e.operands) == 2
                return self.mgr.Minus(self.float_expr(e.operands[0], k, cond=cond, action=action),
                                      self.float_expr(e.operands[1], k, cond=cond, action=action))
            elif e.op == "/":
                assert len(e.operands) == 2
                return self.mgr.Div(self.float_expr(e.operands[0], k, cond=cond, action=action),
                                    self.float_expr(e.operands[1], k, cond=cond, action=action))
            else:
                assert False, 'Unknown arithmetic operator: %s' % e
        elif isinstance(e, pddl.PddlTimeVar):
            return self.t_var
        else:
            assert False, 'Unknown arithmetic expression: %s' % e


    def initial_value(self, f):
        init = self.ground_instance.problem.initial_state
        if isinstance(f, pddl.PddlPredicate):
            for x in init:
                if isinstance(x, pddl.PddlApply) and x.function == f:
                    return self.mgr.TRUE()
            return self.mgr.FALSE()
        else:
            # a function
            for x in init:
                if isinstance(x, pddl.PddlArithRelation) and x.left.function == f:
                    assert x.op == '='
                    return self.float_expr(x.right, None)
            # TODO: this is not respecting the PDDL semantics!
            return self.mgr.Real(0)
            #assert False, "Unable to find initial value of function %s" % f

    def time(self, k):
        if k == 0:
            return self.mgr.Real(0)
        else:
            if k not in self.time_vars:
                if self.debug:
                    self.time_vars[k] = self.mgr.Symbol("time_%d" % k, types.REAL)
                else:
                    self.time_vars[k] = \
                            self.mgr.FreshSymbol(types.REAL, template="time_%d")
            return self.time_vars[k]

    def fluent(self, f, k):
        if k == 0:
            return self.initial_value(f)
        else:
            key = (f, k)
            if key not in self.fluent_vars:
                ty = types.BOOL if isinstance(f, pddl.PddlPredicate) else types.REAL
                if self.affected_variables is None or ty != types.BOOL or f in self.affected_variables[k]:
                    if self.debug:
                        self.fluent_vars[key] = \
                                self.mgr.Symbol("%s_%d" % (str(f.name), k), ty)
                    else:
                        self.fluent_vars[key] = \
                            self.mgr.FreshSymbol(ty,
                                                 template="fluent_" + str(f.name) + "_%d")
                else:
                    if ty == types.BOOL:
                        self.fluent_vars[key] = self.fluent(f, k-1)
                    else:
                        self.fluent_vars[key] = self.mgr.Plus(self.fluent(f, k-1),
                                                              self.encode_increment(f, k))
            return self.fluent_vars[key]

    def time_point(self, tp):
        if tp == self.stn.zero:
            return self.mgr.Real(0)
        else:
            if tp not in self.time_point_vars:
                if self.debug:
                    self.time_point_vars[tp] = \
                        self.mgr.Symbol("tp_%s" % tp.name, types.REAL)
                else:
                    self.time_point_vars[tp] = \
                            self.mgr.FreshSymbol(types.REAL,
                                                 template="tp_"+ tp.name +"_%d")
            return self.time_point_vars[tp]

    def parameter(self, p):
        if p not in self.parameter_vars:
            if self.debug:
                self.parameter_vars[p] = \
                    self.mgr.Symbol("param_%s" % p.name, types.REAL)
            else:
                self.parameter_vars[p] = \
                            self.mgr.FreshSymbol(types.REAL,
                                                 template="param_"+ p.name +"_%d")
        return self.parameter_vars[p]

    def time_point_pos(self, tp):
        if tp == self.stn.zero:
            return self.mgr.Real(0)
        else:
            if tp not in self.time_point_pos_vars:
                if self.debug:
                    self.time_point_pos_vars[tp] = \
                           self.mgr.Symbol("pos_%s" % tp.name, types.REAL)
                else:
                    self.time_point_pos_vars[tp] = \
                           self.mgr.FreshSymbol(types.REAL,
                                                template="pos_"+ tp.name +"_%d")
            return self.time_point_pos_vars[tp]

    def condition(self, expr, k, raw=False, action=None):
        if isinstance(expr, pddl.PddlAnd):
            return self.mgr.And(self.condition(x, k, raw=raw, action=action)
                                for x in expr.expressions)
        elif isinstance(expr, pddl.PddlOr):
            return self.mgr.Or(self.condition(x, k, raw=raw, action=action)
                               for x in expr.expressions)
        elif isinstance(expr, pddl.PddlImplies):
            return self.mgr.Implies(self.condition(expr.left, k, raw=raw, action=action),
                           self.condition(expr.right, k, raw=raw))
        elif isinstance(expr, pddl.PddlNot):
            return self.mgr.Not(self.condition(expr.expression, k, raw=raw, action=action))
        elif isinstance(expr, pddl.PddlApply):
            assert not expr.actual_parameters
            if raw:
                return self.fluent(expr.function, k)
            return self.fluent(expr.function, k-1)
        elif isinstance(expr, pddl.PddlBoolConstant):
            return self.mgr.Bool(expr.value)
        elif isinstance(expr, pddl.PddlArithRelation):
            time = k-1
            cond = True
            if raw:
                time = k
                cond = False
            if expr.op == ">":
                return self.mgr.GT(self.float_expr(expr.left, time, cond=cond, action=action),
                                   self.float_expr(expr.right, time, cond=cond, action=action))
            elif expr.op == ">=":
                return self.mgr.GE(self.float_expr(expr.left, time, cond=cond, action=action),
                                   self.float_expr(expr.right, time, cond=cond, action=action))
            elif expr.op == "<":
                return self.mgr.LT(self.float_expr(expr.left, time, cond=cond, action=action),
                                   self.float_expr(expr.right, time, cond=cond, action=action))
            elif expr.op == "<=":
                return self.mgr.LE(self.float_expr(expr.left, time, cond=cond, action=action),
                                   self.float_expr(expr.right, time, cond=cond, action=action))
            elif expr.op == "=":
                return self.mgr.Equals(self.float_expr(expr.left, time, cond=cond, action=action),
                                       self.float_expr(expr.right, time, cond=cond, action=action))
            else:
                assert False, "Unknown arithmetic operator: %s" % expr
        else:
            assert False, "Unknown condition kind: %s" % expr



    def encode_increment(self, f, k):
        total = []
        for a, tps in self.action_instances:
            if isinstance(a, pddl.PddlDurativeAction):
                s, d = tps
                contribution = []
                for e in a.effect:
                    if isinstance(e, pddl.PddlIncrease) and e.function.function == f:
                        contribution.append(self.float_expr(e.expression, k-1, action=tps))
                    elif isinstance(e, pddl.PddlDecrease) and e.function.function == f:
                        contribution.append(self.mgr.Times(self.mgr.Real(-1),
                                                self.float_expr(e.expression, k-1, action=tps)))
                if contribution:
                    value = contribution[0]
                    if len(contribution) > 1:
                        value = self.mgr.Plus(contribution)
                    condition = self.mgr.And(self.mgr.LT(self.time_point_pos(s),
                                                         self.mgr.Real(k)),
                                             self.mgr.GE(self.time_point_pos(d),
                                                         self.mgr.Real(k)))
                    v1 = value.substitute({self.t_var: self.mgr.Minus(self.time(k),
                                                                      self.time_point(s))})
                    v2 = value.substitute({self.t_var: self.mgr.Minus(self.time(k-1),
                                                                      self.time_point(s))})
                    total.append(self.mgr.Ite(condition, self.mgr.Minus(v1, v2),
                                              self.mgr.Real(0)))
        if total:
            return self.mgr.Plus(total)
        return self.mgr.Real(0)

    def get_effect_literals(self, tp):
        literals = []
        if tp.is_durative_extreme():
            action = self.instance.domain.durative_actions[tp.action_name]
            par_obj = [self.instance.problem.objects[x] for x in tp.parameters]
            g_name = tp.action_name
            if par_obj:
                g_name = action.name + "_" + "_".join(x.name for x in par_obj)
            ground_action = self.ground_instance.domain.durative_actions[g_name]

            for e in ground_action.effect:
                if (tp.is_start() and isinstance(e, pddl.PddlAtStart)) or \
                   (tp.is_end() and isinstance(e, pddl.PddlAtEnd)):
                    literals.append(e.expression)

        elif tp.is_instantaneous_action():
            action = self.instance.domain.actions[tp.action_name]
            par_obj = [self.instance.problem.objects[x] for x in tp.parameters]
            g_name = tp.action_name
            if par_obj:
                g_name = action.name + "_" + "_".join(x.name for x in par_obj)
            ground_action = self.ground_instance.domain.actions[g_name]

            literals = ground_action.effect
        elif tp.is_til():
            literals = tp.til
        else:
            raise NotImplementedError("Unknown time-point kind: %s" % tp.kind)
        return literals

    def encode_effects(self, tp, k):
        tps = tp, tp.paired_time_point
        if tps[1] and tps[1].is_start():
            tps = tuple(reversed(tps))
        literals = self.get_effect_literals(tp)
        assertions = []
        add = assertions.append
        affected = set()
        for l in literals:
            if isinstance(l, pddl.PddlIncrease):
                fun = l.function.function
                affected.add(fun)
                fv = self.fluent(fun, k)
                ofv = self.fluent(fun, k-1)
                nfv = self.mgr.Plus(ofv, self.encode_increment(fun, k),
                                    self.float_expr(l.expression, k-1, action=tps))
                add(self.mgr.Equals(fv, nfv))
            elif isinstance(l, pddl.PddlDecrease):
                fun = l.function.function
                affected.add(fun)
                fv = self.fluent(fun, k)
                ofv = self.fluent(fun, k-1)
                nfv = self.mgr.Minus(self.mgr.Plus(ofv,
                                                  self.encode_increment(fun, k)),
                                     self.float_expr(l.expression, k-1, action=tps))
                add(self.mgr.Equals(fv, nfv))
            elif isinstance(l, pddl.PddlAssign):
                fun = l.function.function
                affected.add(fun)
                fv = self.fluent(fun, k)
                add(self.mgr.Equals(fv, self.float_expr(l.expression, k-1, action=tps)))
            elif isinstance(l, pddl.PddlNot):
                pred = l.expression.function
                affected.add(pred)
                add(self.mgr.Not(self.fluent(pred, k)))
            elif isinstance(l, pddl.PddlApply):
                affected.add(l.function)
                add(self.fluent(l.function, k))
            else:
                raise NotImplementedError("Unknown effect kind: %s" % l)

        # Frame condition
        for key in sorted(self.ground_instance.domain.predicates):
            f = self.ground_instance.domain.predicates[key]
            if f not in affected and (self.affected_variables is None or f in self.affected_variables[k]):
                fv = self.fluent(f, k)
                ofv = self.fluent(f, k-1)
                add(self.mgr.Iff(fv, ofv))
        for key in sorted(self.ground_instance.domain.functions):
            f = self.ground_instance.domain.functions[key]
            if f not in affected:
                fv = self.fluent(f, k)
                ofv = self.fluent(f, k-1)
                add(self.mgr.Equals(fv,
                                    self.mgr.Plus(ofv,
                                                  self.encode_increment(f, k))))
        return self.mgr.And(assertions)

    def encode_continuous_function(self, f, k):
        total = []
        for a, tps in self.action_instances:
            if isinstance(a, pddl.PddlDurativeAction):
                s, d = tps
                if k >= min(self.possible_positions[s]) and \
                   k < max(self.possible_positions[d]):
                    contribution = []
                    for e in a.effect:
                        if isinstance(e, pddl.PddlIncrease) and e.function.function == f:
                            c = self.float_expr(e.expression, k, action=tps)
                            contribution.append(c)
                        elif isinstance(e, pddl.PddlDecrease) and e.function.function == f:
                            c = self.float_expr(e.expression, k, action=tps)
                            contribution.append(self.mgr.Times(self.mgr.Real(-1), c))
                    if contribution:
                        value = contribution[0]
                        if len(contribution) > 1:
                            value = self.mgr.Plus(contribution)
                        cond = self.mgr.And(self.mgr.LE(self.time_point_pos(s),
                                                        self.mgr.Real(k)),
                                            self.mgr.GT(self.time_point_pos(d),
                                                        self.mgr.Real(k)))
                        offset = self.mgr.Minus(self.time(k), self.time_point(s))
                        v1 = value.substitute({self.t_var: self.mgr.Plus(self.t_var,
                                                                         offset)})
                        v2 = value.substitute({self.t_var: self.mgr.Minus(self.time(k),
                                                                          self.time_point(s))})
                        total.append(self.mgr.Ite(cond, self.mgr.Minus(v1, v2),
                                                  self.mgr.Real(0)))
        if total:
            total.append(self.fluent(f, k))
            return self.mgr.Plus(total)
        return self.fluent(f, k)

    def continuous_conditions(self, expr, k, action=None):
        cond = self.filter_condition(expr, k, 'all', raw=True, action=action).simplify()
        if cond.is_true():
            return cond
        else:
            subs = {self.fluent(f, k): self.encode_continuous_function(f, k)
                    for f in self.ground_instance.domain.functions.values()}
            return cond.substitute(subs)

    def filter_condition(self, expr, k, time, raw=False, action=None):
        if isinstance(expr, pddl.PddlAnd):
            return self.mgr.And(self.filter_condition(x, k, time, raw=raw, action=action)
                                for x in expr.expressions)
        elif isinstance(expr, pddl.PddlAtStart):
            if time == "start":
                return self.condition(expr.expression, k, raw=raw, action=action)
            return self.mgr.TRUE()
        elif isinstance(expr, pddl.PddlAtEnd):
            if time == "end":
                return self.condition(expr.expression, k, raw=raw, action=action)
            return self.mgr.TRUE()
        elif isinstance(expr, pddl.PddlOverAll):
            if time == "all":
                return self.condition(expr.expression, k, raw=raw, action=action)
            return self.mgr.TRUE()
        else:
            assert False, "Unsupported condition expression %s" % expr

    def is_mutex(self, a, b):
        pre_a, add_a, del_a, L_a, R_a, L_star_a = self.get_tp_sets(a)
        pre_b, add_b, del_b, L_b, R_b, L_star_b = self.get_tp_sets(b)

        eff_a = add_a | del_a
        eff_b = add_b | del_b

        if len(pre_a & eff_b) > 0:
            return True
        if len(pre_b & eff_a) > 0:
            return True

        if len(add_a & del_b) > 0:
            return True
        if len(add_b & del_a) > 0:
            return True

        if len(L_a & R_b) > 0:
            return True
        if len(L_b & R_a) > 0:
            return True

        if len((L_a & L_b) - (L_star_a & L_star_b)) > 0:
            return True

        return False

    def get_pddl_cone(self, expr, propositional=None, time=None, polarity=None,
                      side=None, additive_only=False):
        if isinstance(expr, pddl.PddlAnd) or isinstance(expr, pddl.PddlOr):
            return set(y for child in expr.expressions
                       for y in self.get_pddl_cone(child,
                                                   propositional=propositional,
                                                   time=time,
                                                   polarity=polarity, side=side,
                                                   additive_only=additive_only))
        elif isinstance(expr, pddl.PddlAtStart):
            if time is None or time == "start":
                return self.get_pddl_cone(expr.expression, time=time,
                                          propositional=propositional,
                                          polarity=polarity, side=side,
                                          additive_only=additive_only)
            return set()
        elif isinstance(expr, pddl.PddlAtEnd):
            if time is None or time == "end":
                return self.get_pddl_cone(expr.expression, time=time,
                                          propositional=propositional,
                                          polarity=polarity, side=side,
                                          additive_only=additive_only)
            return set()
        elif isinstance(expr, pddl.PddlOverAll):
            if time is None or time == "all":
                return self.get_pddl_cone(expr.expression, time=time,
                                          propositional=propositional,
                                          polarity=polarity, side=side,
                                          additive_only=additive_only)
            return set()
        elif isinstance(expr, pddl.PddlNot):
            if polarity is None or polarity == "negative":
                np = None
                if polarity == "negative":
                    np = "positive"
                return self.get_pddl_cone(expr.expression, time=time,
                                          propositional=propositional,
                                          polarity=np, side=side,
                                          additive_only=additive_only)
            return set()
        elif isinstance(expr, pddl.PddlApply):
            if (polarity is None or polarity == "positive") and \
               (propositional is None or propositional is True):
                return set([expr.function.name])
            return set()
        elif isinstance(expr, pddl.PddlIncrease) or isinstance(expr, pddl.PddlDecrease):
            res = set()
            if propositional is True:
                return res
            if side is None or side == "left":
                res |= self.get_pddl_cone(expr.function, time=time,
                                          propositional=None,
                                          polarity=polarity, side=side,
                                          additive_only=additive_only)
            if side is None or side == "right":
                res |= self.get_pddl_cone(expr.expression, time=time,
                                          propositional=None,
                                          polarity=polarity, side=side,
                                          additive_only=additive_only)
            return res
        elif isinstance(expr, pddl.PddlAssign):
            res = set()
            if propositional is True:
                return res
            if not additive_only:
                if side is None or side == "left":
                    res |= self.get_pddl_cone(expr.function, time=time,
                                          propositional=None,
                                          polarity=polarity, side=side,
                                          additive_only=additive_only)
                if side is None or side == "right":
                    res |= self.get_pddl_cone(expr.expression, time=time,
                                          propositional=None,
                                          polarity=polarity, side=side,
                                          additive_only=additive_only)
            return res
        elif isinstance(expr, pddl.PddlArithRelation):
            if additive_only is True or propositional is True or \
               (side is not None and side != "right"):
                return set()
            else:
                res = set()
                res |= self.get_pddl_cone(expr.left, time=time,
                                          propositional=None,
                                          polarity=polarity, side=side,
                                          additive_only=additive_only)
                res |= self.get_pddl_cone(expr.right, time=time,
                                          propositional=None,
                                          polarity=polarity, side=side,
                                          additive_only=additive_only)
                return res
        elif isinstance(expr, pddl.PddlArithOperator):
            return set(y for child in expr.operands
                       for y in self.get_pddl_cone(child,
                                                   propositional=propositional,
                                                   time=time,
                                                   polarity=polarity, side=side,
                                                   additive_only=additive_only))
        elif isinstance(expr, pddl.PddlTimeVar) or \
             isinstance(expr, pddl.PddlBoolConstant) or \
             isinstance(expr, pddl.PddlFloatConstant) or \
             isinstance(expr, pddl.PddlDurationVar) or \
             isinstance(expr, pddl.PddlSynthParam):
            return set()
        else:
            assert False, "Unsupported condition expression %s" % expr

    def get_tp_sets(self, a):
        precondition = None
        time = None
        effects = None

        if a.is_durative_extreme():
            action = self.instance.domain.durative_actions[a.action_name]
            par_obj = [self.instance.problem.objects[x] for x in a.parameters]
            g_name = a.action_name
            if par_obj:
                g_name = action.name + "_" + "_".join(x.name for x in par_obj)
            ground_action = self.ground_instance.domain.durative_actions[g_name]
            precondition = ground_action.condition
            effects = ground_action.effect
            if a.is_start():
                time = "start"
            else:
                time = "end"
        elif a.is_instantaneous_action():
            action = self.instance.domain.actions[a.action_name]
            par_obj = [self.instance.problem.objects[x] for x in a.parameters]
            g_name = a.action_name
            if par_obj:
                g_name = action.name + "_" + "_".join(x.name for x in par_obj)
            ground_action = self.ground_instance.domain.actions[g_name]
            precondition = ground_action.precondition
            effects = ground_action.effect
        elif a.is_til():
            effects = a.til
            precondition = pddl.PddlBoolConstant(True)
        else:
            assert False, "Unsupported time point kind: %s" % a

        assert effects is not None

        pre = self.get_pddl_cone(precondition, propositional=True, time=time)
        add = set(x for e in effects for x in self.get_pddl_cone(e, time=time, propositional=True, polarity="positive"))
        del_ = set(x for e in effects for x in self.get_pddl_cone(e, time=time, propositional=True, polarity="negative"))
        L = self.get_pddl_cone(precondition, propositional=False, time=time, side="left") | \
            set(x for e in effects for x in self.get_pddl_cone(e, propositional=False, time=time, side="left"))
        R = self.get_pddl_cone(precondition, propositional=False, time=time, side="right") | \
            set(x for e in effects for x in self.get_pddl_cone(e, propositional=False, time=time, side="right"))
        L_star = set(x for e in effects for x in self.get_pddl_cone(e, propositional=False, time=time, side="left", additive_only=True))

        return pre, add, del_, L, R, L_star

    #     else:
    #         action = self.instance.domain.actions[a.action_name]
    #         par_obj = [self.instance.problem.objects[x] for x in a.parameters]
    #         g_name = a.action_name
    #         if par_obj:
    #             g_name = action.name + "_" + "_".join(x.name for x in par_obj)
    #         ground_action = self.ground_instance.domain.actions[g_name]

    #         pre_a = self.condition(ground_action.precondition, 2).get_free_variables()
    #         ll = [self.condition(l, 2) for l in self.get_effect_literals(a)
    #               if not isinstance(l, pddl.PddlIncrease) and \
    #                  not isinstance(l, pddl.PddlDecrease)]
    #         eff_a = set(x for y in ll for x in y.get_free_variables())
    #     return pre_a, eff_a

    def param_constraints(self):
        res = []

        for p,pv in self.parameter_vars.items():
            lb = p.lower_bound
            if self.assume_positive_params and (lb is None or lb < 0):
                lb = 0
            if lb is not None:
                res.append(self.mgr.GE(pv, self.mgr.Real(lb)))

            if p.upper_bound is not None:
                res.append(self.mgr.LE(pv, self.mgr.Real(p.upper_bound)))

        return res

    def encode_imp(self):
        implication = []
        add = implication.append
        for tp in self.tps_z:
            add(self.mgr.GE(self.time_point(tp), self.mgr.Real(0)))

        for c in self.stn.constraints:
            sv = self.time_point(c.src)
            dv = self.time_point(c.dst)
            diff = self.mgr.Minus(dv, sv)
            if c.lower_bound is not None:
                if isinstance(c.lower_bound, Parameter):
                    add(self.mgr.GE(diff, self.parameter(c.lower_bound)))
                else:
                    add(self.mgr.GE(diff, self.mgr.Real(c.lower_bound)))
            if c.upper_bound is not None:
                if isinstance(c.upper_bound, Parameter):
                    add(self.mgr.LE(diff, self.parameter(c.upper_bound)))
                else:
                    add(self.mgr.LE(diff, self.mgr.Real(c.upper_bound)))

        for i, tpa in enumerate(self.tps_z):
            for tpb in self.tps_z[i+1:]:
                if self.is_mutex(tpa, tpb):
                    assert tpa != tpb
                    add(self.mgr.Not(self.mgr.Equals(self.time_point(tpa),
                                                     self.time_point(tpb))))

        if self.assume_positive_params:
            implication += self.param_constraints()
        return implication

    def encode(self):
        implication = []
        add = implication.append
        for tp in self.tps_z:
            add(self.mgr.GE(self.time_point(tp), self.mgr.Real(0)))
            pv = self.time_point_pos(tp)
            add(self.mgr.Or(self.mgr.Equals(pv, self.mgr.Real(x))
                            for x in self.possible_positions[tp]))

            for i in self.possible_positions[tp]:
                add(self.mgr.Implies(self.mgr.Equals(pv, self.mgr.Real(i)),
                                     self.mgr.Equals(self.time_point(tp),
                                                     self.time(i))))

            for oth in self.tps_z:
                if oth != tp:
                    t = self.mgr.GT(self.time_point(tp), self.time_point(oth))
                    p = self.mgr.GT(self.time_point_pos(tp), self.time_point_pos(oth))
                    add(self.mgr.Implies(t, p))

        add(self.mgr.AllDifferent(self.time_point_pos(tp) for tp in self.tps_z))

        for c in self.stn.constraints:
            sv = self.time_point(c.src)
            dv = self.time_point(c.dst)
            diff = self.mgr.Minus(dv, sv)
            if c.lower_bound is not None:
                if isinstance(c.lower_bound, Parameter):
                    add(self.mgr.GE(diff, self.parameter(c.lower_bound)))
                else:
                    add(self.mgr.GE(diff, self.mgr.Real(c.lower_bound)))
            if c.upper_bound is not None:
                if isinstance(c.upper_bound, Parameter):
                    add(self.mgr.LE(diff, self.parameter(c.upper_bound)))
                else:
                    add(self.mgr.LE(diff, self.mgr.Real(c.upper_bound)))

        for tp in self.tps_z:
            for s in self.possible_positions[tp]:
                add(self.mgr.Implies(self.mgr.Equals(self.time_point_pos(tp),
                                                     self.mgr.Real(s)),
                                     self.encode_effects(tp, s)))


        goal_achievement = self.condition(self.ground_instance.problem.goal, self.H)

        duration_conditions = []
        action_conditions = []
        for a,tps in self.action_instances:
            if isinstance(a, pddl.PddlDurativeAction):
                s, d = tps
                sv = self.time_point(s)
                dv = self.time_point(d)
                diff = self.mgr.Minus(dv, sv)
                for c in a.duration_constraint:
                    assert isinstance(c.left, pddl.PddlDurationVar)
                    if c.op == "=":
                        duration_conditions.append(self.mgr.Equals(diff,
                                                self.float_expr(c.right, 0, action=tps)))
                    elif c.op == ">=":
                        duration_conditions.append(self.mgr.GE(diff,
                                                self.float_expr(c.right, 0, action=tps)))
                    elif c.op == "<=":
                        duration_conditions.append(self.mgr.LE(diff,
                                                self.float_expr(c.right, 0, action=tps)))
                    else:
                        assert False, "Unsupported duration constraint: %s" % c
                for i in self.possible_positions[s]:
                    at_start_cond = self.filter_condition(a.condition, i, 'start', action=tps).simplify()
                    if not at_start_cond.is_true():
                        cond_start = self.mgr.Equals(self.time_point_pos(s),
                                                     self.mgr.Real(i))
                        action_conditions.append(self.mgr.Implies(cond_start,
                                                                  at_start_cond))
                for i in self.possible_positions[d]:
                    at_end_cond = self.filter_condition(a.condition, i, 'end', action=tps).simplify()
                    if not at_end_cond.is_true():
                        cond_end = self.mgr.Equals(self.time_point_pos(d),
                                                   self.mgr.Real(i))
                        action_conditions.append(self.mgr.Implies(cond_end,
                                                                  at_end_cond))
                for i in range(min(self.possible_positions[s]),
                               max(self.possible_positions[d])+1):
                    over_all_cond = self.filter_condition(a.condition, i, 'all', action=tps).simplify()
                    if not over_all_cond.is_true():
                        cond_all = self.mgr.And(self.mgr.GT(self.time_point_pos(d),
                                                            self.mgr.Real(i)),
                                       self.mgr.LT(self.time_point_pos(s),
                                                   self.mgr.Real(i)))
                        action_conditions.append(self.mgr.Implies(cond_all,
                                                                  over_all_cond))
            else:
                assert isinstance(a, pddl.PddlAction)
                t = tps[0]
                for i in self.possible_positions[t]:
                    pre = self.condition(a.precondition, i, action=tps).simplify()
                    if not pre.is_true():
                        cond = self.mgr.Equals(self.time_point_pos(t),
                                               self.mgr.Real(i))
                        action_conditions.append(self.mgr.Implies(cond, pre))


        durative_invariants = []
        for a,tps in self.action_instances:
            if isinstance(a, pddl.PddlDurativeAction):
                s, d = tps
                for i in range(min(self.possible_positions[s]),
                               max(self.possible_positions[d])):
                    cond = self.mgr.And(self.mgr.LE(self.time_point_pos(s),
                                                    self.mgr.Real(i)),
                                        self.mgr.GT(self.time_point_pos(d),
                                                    self.mgr.Real(i)))
                    continuous_conds = self.continuous_conditions(a.condition, i).simplify()
                    if not continuous_conds.is_true():
                        time_range = self.mgr.And(self.mgr.GT(self.t_var,
                                                              self.mgr.Real(0)),
                                         self.mgr.LT(self.t_var, self.mgr.Minus(self.time(i+1),
                                                                                self.time(i))))
                        check = self.mgr.Implies(time_range, continuous_conds)
                        q_cond = self.mgr.ForAll([self.t_var], check)
                        if self.early_elimination:
                            q_cond = qelim(q_cond, solver_name=self.qelim_name)
                            print(q_cond.serialize())
                        durative_invariants.append(self.mgr.Implies(cond, q_cond))

        epsilon_separation = []
        for i, tp1 in enumerate(self.tps_z):
            for tp2 in self.tps_z[i+1:]:
                if self.is_mutex(tp1, tp2):
                    diff = self.mgr.Minus(self.time_point(tp1), self.time_point(tp2))
                    epsilon_separation.append(self.mgr.Or( \
                                self.mgr.GE(diff, self.mgr.Real(self.epsilon)),
                                self.mgr.LE(diff, self.mgr.Real(-1 * self.epsilon))))


        if self.assume_positive_params:
            implication += self.param_constraints()

        return (implication, [('action_conditions', action_conditions),
                              ('epsilon_separation', epsilon_separation),
                              ('duration_conditions', duration_conditions),
                              ('durative_invariants', durative_invariants),
                              ('goal_achievement', [goal_achievement]),
                              ('extra_constraints', self.param_constraints())])


    def synthesize_parameter_region(self, splitting="monolithic"):
        assert splitting in ["monolithic", "partial", "full"]

        if self.debug: print("Starting encoding...")
        assumptions, proof_obligations = self.encode()
        if len(self.parameter_vars) == 0:
            raise ValueError("Unable to synthesize parameter on a fully" \
                             "instantiated problem instance and plan!")

        assume = self.mgr.And(assumptions)
        constraints = self.mgr.And(y for x in proof_obligations for y in x[1])
        if self.debug: print("Done.")

        f1 = self.mgr.And(self.encode_imp())
        v1 = set(f1.get_free_variables())
        for p in self.parameter_vars.values():
            v1.discard(p)

        if self.debug: print("There are %d variables to eliminate" % len(v1))

        if self.debug: print("Pre-checking...")
        if not is_sat(f1, solver_name=self.solver_name):
            return self.mgr.FALSE()
        if self.debug: print("Done.")

        if splitting == "monolithic":
            f2 = self.mgr.Implies(assume, constraints)
            v2 = set(f2.get_free_variables())
            for p in self.parameter_vars.values():
                v2.remove(p)
            formula = self.mgr.And(self.mgr.Exists(v1, f1), self.mgr.ForAll(v2, f2))
            t0 = time.time()
            res = qelim(formula, solver_name=self.qelim_name)
            t1 = time.time()
            if self.debug:
                print("Monolithic in %s seconds" % (t1 - t0))
            return res.simplify()
        else:
            t0 = time.time()
            res1 = qelim(self.mgr.Exists(v1, f1), solver_name=self.qelim_name)
            t1 = time.time()
            if self.debug:
                print("Exists done in %s seconds" % (t1 - t0))
            conj = [res1]
            for name, po in proof_obligations:
                split = po
                if splitting == "partial":
                    split = [self.mgr.And(po)]
                for f in split:
                    if self.learning:
                        learned = conj[-1]
                        check = self.mgr.Or(self.mgr.And(learned, self.mgr.Not(assume)),
                                            self.mgr.And(learned, f))
                    else:
                        check = self.mgr.Implies(assume, f)
                    fv = set(check.get_free_variables())
                    for p in self.parameter_vars.values():
                        fv.discard(p)
                    q_check = self.mgr.ForAll(fv, check)
                    t0 = time.time()
                    conj.append(qelim(q_check, solver_name=self.qelim_name))
                    t1 = time.time()
                    if self.debug:
                        print("%s done in %s seconds" % (name, t1 - t0))
            return self.mgr.And(conj).simplify()

