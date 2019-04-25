import os
import re
import glob
import argparse
import pandas

from itertools import cycle
from matplotlib import pyplot as plt

from pyrobustenvelope.stn import STN, Parameter

pandas.set_option('display.height', 1000)
pandas.set_option('display.max_rows', 500)
pandas.set_option('display.max_columns', 500)
pandas.set_option('display.width', 1000)

DIR=os.path.dirname(os.path.realpath(__file__))
CDIR='/es0/amicheli/pyrobustnessenvelopes'


def get_instances(benchmarks, cwd):
    for d, data in benchmarks.items():
        ddir = data['ddir']
        tplate = ('{dom_id}', '{iid}', data['dom'], data['prob'], data['pstn'])
        for f in glob.glob(ddir + data['match']):
            m = re.match(data['id_match'], f)
            if m is not None:
                iid = m.group(1)
                m = re.match(data['pid_match'], f)
                assert m is not None
                pid = m.group(1)
                yield tuple(x.format(dom_id=d, pid=pid, iid=iid, cwd=cwd) for x in tplate)


def plot(df, logscale=False, fname=None, title="", legend=True):
    plt.figure(figsize=(7, 6))
    plt.title(title, size='large', weight='bold')
    solved_df = df[df['status'] == 'ok']
    linestyles = cycle(['-', '--', '-.', ':'])
    colors = cycle(['b', 'r', 'g', 'm', 'y', 'k', 'c'])
    for (kind,simp), data in solved_df.groupby(['kind', 'simplified']):
        linestyle = next(linestyles)
        color = next(colors)
        vydata = list(data['time'])
        vydata.sort()
        s = ' with Simplification' if simp else ''
        plt.plot(range(1, len(vydata)+1), vydata, label=kind.capitalize() + s,
                 color=color, linestyle=linestyle)

    for simp, data in df.groupby(['simplified']):
        cdf = data[data['first_improvement'].notna()]
        linestyle = next(linestyles)
        color = next(colors)
        vydata = list(cdf['first_improvement'])
        vydata.sort()
        s = ' with Simplification' if simp else ''
        plt.plot(range(1, len(vydata)+1), vydata, label='First Construct Improvement' + s,
                 color=color, linestyle=linestyle)

    plt.xlabel('Instances Solved')
    plt.ylabel('Solving Time')
    if logscale:
        plt.gca().set_yscale('log')

    if legend:
        plt.legend(loc='best')
    plt.tight_layout()
    if fname is not None:
        plt.savefig(fname)

def main():
    parser = argparse.ArgumentParser(description='Expeval')
    parser.add_argument('command', type=str,
                        choices=['make_defaults', 'get_script', 'plot'])
    parser.add_argument('--mission-id', '-m', type=str)
    parser.add_argument('--logscale', '-l', action='store_true')
    parser.add_argument('--show', '-s', action='store_true')
    parser.add_argument('--cluster', '-c', action='store_true')
    args = parser.parse_args()

    rdir = os.path.join(DIR, 'results')

    benchmarks = {'auv': {'vdir': os.path.join(DIR, 'benchmarks/auv/stns'),
                          'sdir': os.path.join(DIR, 'benchmarks/auv/stns_params'),
                          'ddir': os.path.join(DIR, 'benchmarks/auv/stns_params_with_values'),
                          'match': '/*_params',
                          'id_match': r'^.*?stn_plan_(.+?)_params$',
                          'pid_match': r'^.*?stn_plan_(.+?)_params$',
                          'dom': '{cwd}/benchmarks/auv/domains/pandora_domain_strategic_for_validation.pddl',
                          'prob': '{cwd}/benchmarks/auv/problems/{pid}.pddl',
                          'pstn': '{cwd}/benchmarks/auv/stns_params_with_values/stn_plan_{iid}_params',
                          'map': lambda x : x.replace('_params', '_var00')},
                  'generator_linear': {'vdir': os.path.join(DIR, 'benchmarks/generator_linear/output_generator_linear'),
                                       'sdir': os.path.join(DIR, 'benchmarks/generator_linear/pram_plan_generator_linear_stn'),
                                       'ddir': os.path.join(DIR, 'benchmarks/generator_linear/stns_params_with_values'),
                                       'match': '/*pram.stn',
                                       'id_match': r'^.*?output(\d+_\d+)pram.stn$',
                                       'pid_match': r'^.*?output(\d+)_\d+pram.stn$',
                                       'dom': '{cwd}/benchmarks/generator_linear/gen_linear_domain.pddl',
                                       'prob': '{cwd}/benchmarks/generator_linear/gen_linear_prob{pid}.pddl',
                                       'pstn': '{cwd}/benchmarks/generator_linear/stns_params_with_values/output{iid}pram.stn',
                                       'map': lambda x : re.sub(r'^(.*?)_\d+pram.stn$', r'\1_01.stn', x)},
                  'solar_rover': {'vdir': os.path.join(DIR, 'benchmarks/solar-rover_LinearPlans/flexible/STN_plans'),
                                  'sdir': os.path.join(DIR, 'benchmarks/solar-rover_LinearPlans/flexible/STN_plans_pram'),
                                  'ddir': os.path.join(DIR, 'benchmarks/solar-rover_LinearPlans/flexible/stns_params_with_values'),
                                  'match': '/*pram.stn',
                                  'id_match': r'^.*?output(\d+_\d+)pram.stn$',
                                  'pid_match': r'^.*?output(\d+)_\d+pram.stn$',
                                  'dom': '{cwd}/benchmarks/solar-rover_LinearPlans/flexible/solarrover_linear.pddl',
                                  'prob': '{cwd}/benchmarks/solar-rover_LinearPlans/flexible/prob{pid}.pddl',
                                  'pstn': '{cwd}/benchmarks/solar-rover_LinearPlans/flexible/stns_params_with_values/output{iid}pram.stn',
                                  'map': lambda x : re.sub(r'^(.*?)_\d+pram.stn$', r'\1.stn', x)},
                  'robot_delivery': {'vdir': os.path.join(DIR, 'benchmarks/robot_delivery/stns'),
                                     'sdir': os.path.join(DIR, 'benchmarks/robot_delivery/param_stns'),
                                     'ddir': os.path.join(DIR, 'benchmarks/robot_delivery/param_stns'),
                                     'match': '/*.stn',
                                     'id_match': r'^.*?problem_(g\d+_w\d+)_ROSPlan.stn$',
                                     'pid_match': r'^.*?problem_(g\d+_w\d+)_ROSPlan.stn$',
                                     'dom': '{cwd}/benchmarks/robot_delivery/domain_robot_delivery.pddl',
                                     'prob': '{cwd}/benchmarks/robot_delivery/domain/problem_{pid}.pddl',
                                     'pstn': '{cwd}/benchmarks/robot_delivery/param_stns/STN_plan_problem_{iid}_ROSPlan.stn',
                                     'map': None}
    }


    if args.command == 'make_defaults':
        defaults = {}
        def add(pv, vv):
            if isinstance(pv, Parameter):
                if pv in defaults:
                    assert vv == defaults[pv]
                else:
                    defaults[pv] = vv

        for _, data in benchmarks.items():
            vdir, sdir, ddir = data['vdir'], data['sdir'], data['ddir']

            if not os.path.exists(ddir):
                os.mkdir(ddir)

            for stn_file in glob.glob(sdir + data['match']):
                defaults.clear()

                vstn_file = data['map'](os.path.basename(stn_file))
                vstn_file = os.path.join(vdir, vstn_file)
                stn = STN(stn_file)
                vstn = STN(vstn_file)


                for c in stn.constraints:
                    if c.is_parametric():
                        for c1 in vstn.constraints:
                            if c1.src == c.src and c.dst == c1.dst:
                                add(c.lower_bound, c1.lower_bound)
                                add(c.upper_bound, c1.upper_bound)
                                break

                for p, v in defaults.items():
                    p.default = v

                with open(os.path.join(ddir, os.path.basename(stn_file)), 'wt') as f:
                    f.write(str(stn))

    elif args.command == 'get_script':
        cwd = DIR
        if args.cluster:
            cwd = CDIR
        for dom_id, iid, dom, prob, pstn in get_instances(benchmarks, cwd):
            timeout = 3600
            memout = 20000

            for kind in ['construct', 'envelope']:
                for simp in [True, False]:
                    add = ' -c' if (not simp) else ''
                    if kind == 'construct':
                        cmd_str = 'construct {dom} {prob} {pstn} -d -S monolithic -s z3 -q msat_lw' + add
                    else:
                        cmd_str = 'envelope {dom} {prob} {pstn} -d -S partial -s msat -e -q msat_lw' + add

                    nadd = '_simp' if simp else '_nosimp'
                    ofile = dom_id + '_' + iid + '_' + kind + nadd + '.res'
                    logfile = dom_id + '_' + iid + '_' + kind + nadd + '.log'

                    prefix = 'runlim -t {timeout} -s {memout} -o {cwd}/results/{logfile} python3 {cwd}/main.py -o {cwd}/results/{outfile} '
                    if args.cluster:
                        prefix = 'qsub -q es3.q -p -100 -N test-tpack -l "mf=25G,h=korehpc115|korehpc116|korehpc117" -pe smp 2 -S /bin/bash /es0/amicheli/runner.sh /es0/amicheli/myroot/bin/' + prefix

                    cmd = (prefix + cmd_str).format(dom=dom,
                                                    prob=prob,
                                                    pstn=pstn,
                                                    timeout=timeout,
                                                    memout=memout,
                                                    logfile=logfile,
                                                    outfile=ofile,
                                                    cwd=cwd)
                    print(cmd)

    elif args.command == 'plot':
        cwd = DIR
        if args.cluster:
            cwd = CDIR
        df = pandas.DataFrame(columns=['domain', 'instance_id', 'kind', 'simplified', 'status', 'time', 'space', 'first_improvement'])
        for dom_id, iid, _, _, _ in get_instances(benchmarks, cwd):
            for kind in ['construct', 'envelope']:
                for simp in [True, False]:
                    nadd = '_simp' if simp else '_nosimp'
                    ofile = os.path.join(rdir, dom_id + '_' + iid + '_' + kind + nadd + '.res')
                    logfile = os.path.join(rdir, dom_id + '_' + iid + '_' + kind + nadd + '.log')

                    time = None
                    space = None
                    status = 'todo'
                    if os.path.exists(logfile):
                        status = 'running'
                        with open(logfile, 'rt') as f:
                            for l in f:
                                l = l.strip()
                                m = re.match(r'.*?status:\s*(.*)', l)
                                if m is not None:
                                    status = m.group(1)
                                m = re.match(r'.*?time:\s*(.*?) seconds', l)
                                if m is not None:
                                    time = float(m.group(1))
                                m = re.match(r'.*?space:\s*(.*?) MB', l)
                                if m is not None:
                                    space = float(m.group(1))

                    first_improvement = None
                    if kind == 'construct':
                        if os.path.exists(ofile):
                            with open(ofile, 'rt') as f:
                                for l in f:
                                    l = l.strip()
                                    m = re.match(r'.*?First improvement after: (.*?) seconds', l)
                                    if m is not None:
                                        first_improvement = float(m.group(1))
                    df = df.append({'domain': dom_id,
                                    'instance_id' : iid,
                                    'kind' : kind,
                                    'simplified': simp,
                                    'status' : status,
                                    'time' : time,
                                    'space' : space,
                                    'first_improvement' : first_improvement}, ignore_index=True)
        print(df)
        plot(df, logscale=args.logscale, title = "Construct vs Envelope Overall", fname='all.pdf')
        plot(df[df['domain'] == 'auv'], logscale=args.logscale, title = "Construct vs Envelope AUV", fname='auv.pdf')
        plot(df[df['domain'] == 'solar_rover'], logscale=args.logscale, title = "Construct vs Envelope Solar Rover", fname='solar_rover.pdf', legend=False)
        plot(df[df['domain'] == 'generator_linear'], logscale=args.logscale, title = "Construct vs Envelope Generator Linear", fname='generator_linear.pdf', legend=False)
        plot(df[df['domain'] == 'robot_delivery'], logscale=args.logscale, title = "Construct vs Envelope Robot Delivery", fname='robot_delivery.pdf', legend=False)

        if args.show:
            plt.show()
        plt.close('all')



if __name__ == '__main__':
    main()

