import time

from itertools import cycle

from pysmt.shortcuts import Solver, qelim, get_model
from pysmt.oracles import get_logic


class ConstructAlgorithm(object):

    def __init__(self, instance, stn, enc, debug=None, splitting=None,
                 solver=None, qelim_name=None, bound=1):
        self.instance = instance
        self.stn = stn
        self.enc = enc
        self.debug = debug
        self.splitting = splitting
        self.solver = solver
        self.qelim_name = qelim_name
        self.bound = bound

        self.s1 = None
        self.vsolvers = None

    def p(self, s, *args):
        if self.debug:
            if args:
                self.debug.write(s % args)
            else:
                self.debug.write(s)
            self.debug.write('\n')
            self.debug.flush()

    def check_in_region(self, R):
        self.p("\nChecking new rectangle...")
        t0 = time.time()
        mgr = self.enc.mgr
        self.s1.push()
        for  (_, p), (l,u) in R.items():
            self.s1.add_assertion(mgr.GE(p, mgr.Real(l)))
            self.s1.add_assertion(mgr.LE(p, mgr.Real(u)))
        check1 = self.s1.solve()
        self.s1.pop()
        t1 = time.time()
        self.p(" -> Check non-trivial done in %.2f seconds", (t1 - t0))
        if check1:
            return False

        t0 = time.time()
        for n, s in self.vsolvers.items():
            s.push()
            for  (_, p), (l,u) in R.items():
                s.add_assertion(mgr.GE(p, mgr.Real(l)))
                s.add_assertion(mgr.LE(p, mgr.Real(u)))
            self.p(" -> Checking %s", n)
            check2 = s.solve()
            s.pop()
            if check2:
                t1 = time.time()
                self.p(" -> Check in-region done in %.2f seconds", (t1 - t0))
                return False
        t1 = time.time()
        self.p(" -> Check in-region done in %.2f seconds", (t1 - t0))
        return True


    def print_rectangle(self, R, title=None, deltas=None,
                        flexibility=None, volume=None):
        if title:
            self.p(('== %s ' % title) + '='*(80-(len(title) + 4)))
        else:
            self.p('='*80)

        for p, (l, u) in R.items():
            dt = ''
            if deltas is not None:
                dt = 'delta: %.3f' % deltas[p]
            self.p('%s in [%.3f, %.3f] %s' % (p[0].name, l, u, dt))
        self.p('')
        if flexibility is not None:
            self.p(' - Flexibility: %.3f' % flexibility)
        if volume is not None:
            self.p(' - Volume: %E' % volume)
        self.p('='*80)

    def get_flexibility_measures(self, R):
        flexibility = 0
        volume = 1
        for (l, u) in R.values():
            flexibility += (u-l)
            volume *= (u-l)
        return flexibility, volume


    def compute_v(self, X):
        self.p("computing V...")
        t0 = time.time()
        mgr = self.enc.mgr
        V = qelim(mgr.Exists(X, mgr.And(self.enc.encode_imp())), solver_name=self.qelim_name)
        t1 = time.time()
        self.p(" -> Done in %s seconds", (t1 - t0))
        return V


    def find_starting_point(self, X, V, assume, constraints):
        self.p("Finding starting point...")
        t0 = time.time()
        mgr = self.enc.mgr
        all_assigned = True
        subs = {}
        R = {}
        for p, pv in self.enc.parameter_vars.items():
            if p.default is not None:
                R[(p,pv)] = [p.default, p.default]
                subs[pv] = mgr.Real(p.default)
            else:
                all_assigned = False
        if not all_assigned:
            self.p(' -> Not all parameters are assigned, finding missing values...')
            m = get_model(mgr.ForAll(X, mgr.Or(mgr.And(V, mgr.Not(assume)),
                                               mgr.And(V, constraints)).substitute(subs)),
                          solver_name='z3')
            for p in self.enc.parameter_vars.items():
                v = m[p[1]].constant_value()
                R[p] = [v, v]

        t1 = time.time()
        self.p(" -> Done in %s seconds" % (t1 - t0))
        return R


    def prepare_solvers(self, V, assume, constraints, proof_obligations):
        self.p("Building Solvers...")
        t0 = time.time()
        mgr = self.enc.mgr

        self.s1 = Solver(name=self.solver)
        self.s1.add_assertion(mgr.Not(V))

        self.vsolvers = {}
        if self.splitting == 'monolithic':
            s2 = Solver(name=self.solver)
            s2.add_assertion(assume)
            s2.add_assertion(mgr.Not(constraints))
            self.vsolvers['monolithic'] = s2
        else:
            for name, po in proof_obligations:
                split = po
                n = name + " {id}"
                if self.splitting == "partial":
                    split = [mgr.And(po)]
                    n = name
                for i,f in enumerate(split):
                    nn = n.format(id=i)
                    phi = mgr.And(assume, mgr.Not(f))
                    l = get_logic(phi)
                    if l.quantifier_free:
                        s2 = Solver(name=self.solver)
                        self.p('%s uses %s', nn, self.solver)
                    else:
                        self.p('%s uses z3', nn)
                        s2 = Solver(name='z3')
                    s2.add_assertion(phi)
                    self.vsolvers[nn] = s2
        t1 = time.time()
        self.p(" -> Done in %s seconds" % (t1 - t0))


    def run(self, rectangle_callback=None):
        t_init = time.time()

        mgr = self.enc.mgr
        assumptions, proof_obligations = self.enc.encode()
        assume = mgr.And(assumptions)
        constraints = mgr.And(y for x in proof_obligations for y in x[1])

        X = set(mgr.And(assume, constraints).get_free_variables())
        for p in self.enc.parameter_vars.values():
            X.discard(p)

        V = self.compute_v(X)

        R = self.find_starting_point(X, V, assume, constraints)

        self.prepare_solvers(V, assume, constraints, proof_obligations)

        deltas = {p:(max(R[p][0], 1) * p[0].weight) for p in R}
        can_grow = {p:True for p in R}
        gammas = cycle(list(R))
        choices = {p:[1, -1] for p in R}
        self.print_rectangle(R, title='Initial Rectangle')
        first_flex = False
        current_flexibility = -1
        t_flex = None
        while any(x >= self.bound for x in deltas.values()):
            flexibility, volume = self.get_flexibility_measures(R)
            if self.debug:
                self.print_rectangle(R, title='Current Rectangle', deltas=deltas,
                                     flexibility=flexibility, volume=volume)
                if not first_flex and flexibility > 0:
                    t_flex = time.time()
                    self.p('First improvement after: %.2f seconds', (t_flex-t_init))
                    first_flex = True

            if rectangle_callback is not None and flexibility > current_flexibility:
                rectangle_callback(R)

            gamma = next(gammas)
            delta = deltas[gamma]
            if len(choices[gamma]) > 0:
                m = choices[gamma][0]
                R1 = {k: [l, u] for k, (l, u) in R.items()}
                if m == 0:
                    R1[gamma][0] -= delta
                    R1[gamma][1] += delta
                else:
                    R1[gamma][1 if m > 0 else 0] += m * delta
                if self.check_in_region(R1):
                    R = R1
                    if can_grow[gamma]:
                        deltas[gamma] *= 2
                else:
                    choices[gamma].remove(m)

                if len(choices[gamma]) == 0 and deltas[gamma] >= self.bound:
                    can_grow[gamma] = False
                    deltas[gamma] = deltas[gamma] / 2
                    choices[gamma] = [1, -1]
            current_flexibility = flexibility

        if self.debug:
            t_end = time.time()
            flexibility, volume = self.get_flexibility_measures(R)
            self.print_rectangle(R, title='Final Result',
                                 flexibility=flexibility, volume=volume)
            self.p('Total construction time: %.2f seconds', (t_end-t_init))
            if t_flex:
                self.p('First improvement after: %.2f seconds', (t_flex-t_init))

        return {p:v for (p, _), v in R.items()}
