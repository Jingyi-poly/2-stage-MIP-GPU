#include <scip/scip.h>
#include <scip/scipdefplugins.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "config.h"
#include "io/pdptw_reader.h"
#include "io/sol_reader.h"

namespace {

constexpr double kOverloadPenalty = 20000.0;

struct Options {
    std::string instance;
    std::string trace = "pdptw_scip_trace.csv";
    int scenarios = 5;
    int seed = 42;
    int routes = -1;
    int requests = -1;
    int threads = 1;
    double time_limit = 60.0;
    double cap_scale = 1.0;
    std::string warm_start = "auto";
};

void Usage(const char* argv0) {
    std::cout << "Usage: " << argv0 << " --instance FILE_OR_NAME [options]\n"
              << "  --scenarios N --seed N --time-limit SEC --cap-scale X\n"
              << "  --routes K --requests R --threads N --trace FILE\n"
              << "  --warm-start auto|off|FILE  (default: auto BKS if present)\n\n"
              << "Model assumptions: first R pickup-delivery pairs (or all pairs); identical\n"
              << "vehicles; one depot; hard time windows; each pair is assigned to one vehicle\n"
              << "with pickup before delivery; at most K nonempty routes. Objective is route\n"
              << "distance + 20000 times the scenario mean of positive load-capacity at every\n"
              << "visited route prefix. Loads may exceed capacity and are penalized, not forbidden.\n";
}

Options Parse(int argc, char** argv) {
    Options o;
    auto need = [&](int& i) -> std::string {
        if (++i >= argc) throw std::runtime_error(std::string("missing value after ") + argv[i - 1]);
        return argv[i];
    };
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--instance") o.instance = need(i);
        else if (a == "--trace") o.trace = need(i);
        else if (a == "--scenarios") o.scenarios = std::stoi(need(i));
        else if (a == "--seed") o.seed = std::stoi(need(i));
        else if (a == "--time-limit") o.time_limit = std::stod(need(i));
        else if (a == "--cap-scale") o.cap_scale = std::stod(need(i));
        else if (a == "--routes") o.routes = std::stoi(need(i));
        else if (a == "--requests") o.requests = std::stoi(need(i));
        else if (a == "--threads") o.threads = std::stoi(need(i));
        else if (a == "--warm-start") o.warm_start = need(i);
        else if (a == "--help" || a == "-h") { Usage(argv[0]); std::exit(0); }
        else throw std::runtime_error("unknown option: " + a);
    }
    if (o.instance.empty()) throw std::runtime_error("--instance is required");
    if (o.scenarios <= 0 || o.time_limit <= 0 || o.cap_scale <= 0 || o.threads <= 0)
        throw std::runtime_error("scenarios, time-limit, cap-scale and threads must be positive");
    return o;
}

std::string ResolveInstance(const std::string& input) {
    namespace fs = std::filesystem;
    if (fs::exists(input)) return input;
    fs::path p = fs::path(AMDAHL_DIR) / "data" / "instance" / input;
    if (!p.has_extension()) p += ".txt";
    if (fs::exists(p)) return p.string();
    throw std::runtime_error("instance not found: " + input);
}

std::optional<std::string> ResolveWarmStart(const Options& o, const std::string& instance) {
    namespace fs = std::filesystem;
    if (o.warm_start == "off" || o.warm_start == "none") return std::nullopt;

    if (o.warm_start != "auto") {
        fs::path p = o.warm_start;
        if (!fs::exists(p)) {
            p = fs::path(AMDAHL_DIR) / "data" / "bks" / o.warm_start;
            if (!p.has_extension()) p += ".txt";
        }
        if (!fs::exists(p)) throw std::runtime_error("warm-start solution not found: " + o.warm_start);
        return p.string();
    }

    const fs::path p = fs::path(AMDAHL_DIR) / "data" / "bks" /
                       (fs::path(instance).stem().string() + ".txt");
    if (fs::exists(p)) return p.string();
    return std::nullopt;
}

void Check(SCIP_RETCODE rc, const char* what) {
    if (rc != SCIP_OKAY) throw std::runtime_error(std::string(what) + " failed (SCIP code " + std::to_string(rc) + ")");
}

struct TraceData {
    std::ofstream out;
    double last_primal = SCIP_INVALID;
    double last_dual = SCIP_INVALID;
    bool accept_events = true;
};

void WriteTrace(SCIP* scip, TraceData* d, const char* event_name, bool force = false) {
    const double primal = SCIPgetPrimalbound(scip);
    const double dual = SCIPgetDualbound(scip);
    if (!force && SCIPisEQ(scip, primal, d->last_primal) && SCIPisEQ(scip, dual, d->last_dual)) return;
    d->last_primal = primal;
    d->last_dual = dual;
    auto value = [&](double x) {
        if (SCIPisInfinity(scip, std::fabs(x))) d->out << (x < 0 ? "-inf" : "inf");
        else d->out << std::setprecision(15) << x;
    };
    d->out << std::fixed << std::setprecision(6) << SCIPgetSolvingTime(scip) << ',' << event_name << ',';
    value(primal); d->out << ','; value(dual); d->out << ',';
    const double gap = SCIPgetGap(scip);
    if (SCIPisInfinity(scip, gap)) d->out << "inf"; else d->out << std::setprecision(15) << gap;
    d->out << '\n';
    d->out.flush();
}

SCIP_DECL_EVENTEXEC(TraceExec) {
    auto* d = reinterpret_cast<TraceData*>(SCIPeventhdlrGetData(eventhdlr));
    if (!d->accept_events) return SCIP_OKAY;
    WriteTrace(scip, d, (SCIPeventGetType(event) & SCIP_EVENTTYPE_BESTSOLFOUND) ? "incumbent" : "dual");
    return SCIP_OKAY;
}
SCIP_DECL_EVENTINITSOL(TraceInit) {
    SCIP_CALL(SCIPcatchEvent(scip, SCIP_EVENTTYPE_GAPUPDATED | SCIP_EVENTTYPE_BESTSOLFOUND,
                             eventhdlr, nullptr, nullptr));
    return SCIP_OKAY;
}
SCIP_DECL_EVENTEXITSOL(TraceExit) {
    SCIP_CALL(SCIPdropEvent(scip, SCIP_EVENTTYPE_GAPUPDATED | SCIP_EVENTTYPE_BESTSOLFOUND,
                            eventhdlr, nullptr, -1));
    return SCIP_OKAY;
}

SCIP_VAR* AddVar(SCIP* scip, const std::string& name, double lb, double ub, double obj, SCIP_VARTYPE type) {
    SCIP_VAR* v = nullptr;
    Check(SCIPcreateVarBasic(scip, &v, name.c_str(), lb, ub, obj, type), "create variable");
    Check(SCIPaddVar(scip, v), "add variable");
    return v;
}

void AddLinear(SCIP* scip, const std::string& name, const std::vector<std::pair<SCIP_VAR*, double>>& terms,
               double lhs, double rhs) {
    SCIP_CONS* c = nullptr;
    Check(SCIPcreateConsBasicLinear(scip, &c, name.c_str(), 0, nullptr, nullptr, lhs, rhs), "create constraint");
    for (const auto& [v, coef] : terms) Check(SCIPaddCoefLinear(scip, c, v, coef), "add coefficient");
    Check(SCIPaddCons(scip, c), "add constraint");
    Check(SCIPreleaseCons(scip, &c), "release constraint");
}

bool AddBksWarmStart(SCIP* scip, const PDPTW& p, const std::string& path, int R, int N, int K,
                     int scenarios, double cap, const std::vector<double>& qmax,
                     const std::vector<SCIP_VAR*>& x, const std::vector<SCIP_VAR*>& y,
                     const std::vector<SCIP_VAR*>& t, const std::vector<SCIP_VAR*>& q,
                     const std::vector<SCIP_VAR*>& z, const std::vector<SCIP_VAR*>& use,
                     std::string& rejection_reason, double& objective) {
    Solution bks(p);
    if (!ReadSol(path, bks)) {
        rejection_reason = "reader rejected solution";
        return false;
    }

    std::vector<const std::vector<idx_t>*> routes;
    for (const auto& route : bks.routes_) {
        if (route.size() > 2) routes.push_back(&route);
    }
    if (static_cast<int>(routes.size()) > K) {
        rejection_reason = "BKS uses " + std::to_string(routes.size()) + " routes but model has " +
                           std::to_string(K);
        return false;
    }

    std::vector<int> seen(N, 0), node_route(N, -1), node_pos(N, -1);
    for (int k = 0; k < static_cast<int>(routes.size()); ++k) {
        const auto& route = *routes[k];
        if (route.front() != 0 || route.back() != 0) {
            rejection_reason = "a route does not start and end at the depot";
            return false;
        }
        for (int pos = 1; pos + 1 < static_cast<int>(route.size()); ++pos) {
            const int node = route[pos];
            if (node <= 0 || node >= N) {
                rejection_reason = "BKS contains a node outside the selected request subset";
                return false;
            }
            if (++seen[node] != 1) {
                rejection_reason = "BKS visits a selected node more than once";
                return false;
            }
            node_route[node] = k;
            node_pos[node] = pos;
        }
    }
    for (int i = 1; i < N; ++i) {
        if (seen[i] != 1) {
            rejection_reason = "BKS does not visit every selected node exactly once";
            return false;
        }
    }
    for (int r = 0; r < R; ++r) {
        const int pickup = 2 * r + 1, delivery = pickup + 1;
        if (node_route[pickup] != node_route[delivery] || node_pos[pickup] >= node_pos[delivery]) {
            rejection_reason = "BKS violates pickup-delivery pairing or precedence";
            return false;
        }
    }

    auto xid = [=](int k, int i, int j) { return (k * N + i) * N + j; };
    auto yid = [=](int k, int i) { return k * N + i; };
    auto qid = [=](int k, int i, int s) { return (k * N + i) * scenarios + s; };
    SCIP_SOL* sol = nullptr;
    Check(SCIPcreateSol(scip, &sol, nullptr), "create warm-start solution");
    auto set = [&](SCIP_VAR* var, double value) {
        Check(SCIPsetSolVal(scip, sol, var, value), "set warm-start value");
    };

    // Assign every model variable.  In particular, continuous values are not left
    // for SCIP to infer, so the candidate is a complete, directly checkable solution.
    for (int k = 0; k < K; ++k) {
        set(use[k], 0.0);
        for (int i = 1; i < N; ++i) {
            set(y[yid(k, i)], 0.0);
            set(t[yid(k, i)], GetEarliestStartTime(p, i));
            for (int s = 0; s < scenarios; ++s) {
                set(q[qid(k, i, s)], 0.0);
                set(z[qid(k, i, s)], 0.0);
            }
        }
        for (int i = 0; i < N; ++i)
            for (int j = 0; j < N; ++j)
                if (i != j) set(x[xid(k, i, j)], 0.0);
    }

    objective = 0.0;
    for (int k = 0; k < static_cast<int>(routes.size()); ++k) {
        const auto& route = *routes[k];
        set(use[k], 1.0);
        double departure = GetEarliestStartTime(p, 0) + p.task_service_times[0];
        int previous = 0;
        std::vector<double> load(scenarios, 0.0);
        for (int pos = 1; pos < static_cast<int>(route.size()); ++pos) {
            const int node = route[pos];
            set(x[xid(k, previous, node)], 1.0);
            objective += GetDistance(p, previous, node);
            if (node == 0) {
                const double depot_arrival = departure + GetDistance(p, previous, 0);
                if (depot_arrival > GetLatestStartTime(p, 0) + 1e-6) {
                    Check(SCIPfreeSol(scip, &sol), "free infeasible warm-start solution");
                    rejection_reason = "BKS returns to the depot after its time window";
                    return false;
                }
                previous = 0;
                continue;
            }

            set(y[yid(k, node)], 1.0);
            const double arrival = departure + GetDistance(p, previous, node);
            const double start = std::max(arrival, GetEarliestStartTime(p, node));
            if (start > GetLatestStartTime(p, node) + 1e-6) {
                Check(SCIPfreeSol(scip, &sol), "free infeasible warm-start solution");
                rejection_reason = "BKS violates a time window";
                return false;
            }
            set(t[yid(k, node)], start);
            departure = start + p.task_service_times[node];
            for (int s = 0; s < scenarios; ++s) {
                load[s] += p.task_demands[node][s];
                if (load[s] < -1e-6 || load[s] > qmax[s] + 1e-6) {
                    Check(SCIPfreeSol(scip, &sol), "free infeasible warm-start solution");
                    rejection_reason = "BKS produced an invalid scenario load";
                    return false;
                }
                const double overload = std::max(0.0, load[s] - cap);
                set(q[qid(k, node, s)], load[s]);
                set(z[qid(k, node, s)], overload);
                objective += kOverloadPenalty * overload / scenarios;
            }
            previous = node;
        }
    }

    SCIP_Bool feasible = FALSE;
    Check(SCIPcheckSol(scip, sol, TRUE, TRUE, TRUE, TRUE, TRUE, &feasible),
          "check warm-start solution");
    if (!feasible) {
        Check(SCIPfreeSol(scip, &sol), "free infeasible warm-start solution");
        rejection_reason = "SCIP feasibility check rejected the complete candidate";
        return false;
    }

    SCIP_Bool stored = FALSE;
    Check(SCIPaddSol(scip, sol, &stored), "add warm-start solution");
    Check(SCIPfreeSol(scip, &sol), "free warm-start solution");
    if (!stored) rejection_reason = "candidate is feasible but SCIP did not retain it";
    return stored == TRUE;
}

}  // namespace

int main(int argc, char** argv) {
  SCIP* scip = nullptr;
  std::vector<SCIP_VAR*> vars;
  try {
    const Options o = Parse(argc, argv);
    PDPTW p;
    const std::string instance = ResolveInstance(o.instance);
    if (!ReadPDPTW(instance, p, ProblemFormat::kLiLim, o.scenarios, static_cast<uint32_t>(o.seed), false))
        throw std::runtime_error("failed to read instance");
    const int available = (p.task_num - 1) / 2;
    const int R = o.requests < 0 ? available : std::min(o.requests, available);
    const int N = 2 * R + 1;
    const int K = o.routes < 0 ? std::min(p.vehicle_num, R) : o.routes;
    if (R <= 0 || K <= 0) throw std::runtime_error("requests and routes must be positive");
    const double cap = p.vehicle_capacity * o.cap_scale;

    Check(SCIPcreate(&scip), "SCIPcreate");
    Check(SCIPincludeDefaultPlugins(scip), "include plugins");
    Check(SCIPcreateProbBasic(scip, "stochastic_pdptw"), "create problem");
    Check(SCIPsetRealParam(scip, "limits/time", o.time_limit), "set time limit");
    Check(SCIPsetIntParam(scip, "parallel/maxnthreads", o.threads), "set threads");
    Check(SCIPsetIntParam(scip, "randomization/randomseedshift", o.seed), "set seed");
    Check(SCIPsetObjsense(scip, SCIP_OBJSENSE_MINIMIZE), "set objective sense");

    TraceData trace;
    trace.out.open(o.trace);
    if (!trace.out) throw std::runtime_error("cannot open trace: " + o.trace);
    trace.out << "time_s,event,incumbent,dual_bound,proof_gap\n";
    SCIP_EVENTHDLR* eh = nullptr;
    Check(SCIPincludeEventhdlrBasic(scip, &eh, "anytimetrace", "CSV primal/dual trace", TraceExec,
          reinterpret_cast<SCIP_EVENTHDLRDATA*>(&trace)), "include event handler");
    Check(SCIPsetEventhdlrInitsol(scip, eh, TraceInit), "set event init");
    Check(SCIPsetEventhdlrExitsol(scip, eh, TraceExit), "set event exit");

    auto xid = [=](int k, int i, int j) { return (k * N + i) * N + j; };
    auto yid = [=](int k, int i) { return k * N + i; };
    auto qid = [=](int k, int i, int s) { return (k * N + i) * o.scenarios + s; };
    std::vector<SCIP_VAR*> x(K * N * N, nullptr), y(K * N, nullptr), t(K * N, nullptr), use(K, nullptr);
    std::vector<SCIP_VAR*> q(K * N * o.scenarios, nullptr), z(K * N * o.scenarios, nullptr);
    std::vector<double> qmax(o.scenarios, 0.0);
    for (int s = 0; s < o.scenarios; ++s)
        for (int i = 1; i < N; ++i) qmax[s] += std::max(0, p.task_demands[i][s]);

    for (int k = 0; k < K; ++k) {
      use[k] = AddVar(scip, "use_" + std::to_string(k), 0, 1, 0, SCIP_VARTYPE_BINARY); vars.push_back(use[k]);
      for (int i = 1; i < N; ++i) {
        y[yid(k,i)] = AddVar(scip, "y_"+std::to_string(k)+"_"+std::to_string(i), 0, 1, 0, SCIP_VARTYPE_BINARY); vars.push_back(y[yid(k,i)]);
        t[yid(k,i)] = AddVar(scip, "t_"+std::to_string(k)+"_"+std::to_string(i), GetEarliestStartTime(p,i), GetLatestStartTime(p,i), 0, SCIP_VARTYPE_CONTINUOUS); vars.push_back(t[yid(k,i)]);
        for (int s = 0; s < o.scenarios; ++s) {
          q[qid(k,i,s)] = AddVar(scip, "q_"+std::to_string(k)+"_"+std::to_string(i)+"_"+std::to_string(s), 0, qmax[s], 0, SCIP_VARTYPE_CONTINUOUS); vars.push_back(q[qid(k,i,s)]);
          z[qid(k,i,s)] = AddVar(scip, "z_"+std::to_string(k)+"_"+std::to_string(i)+"_"+std::to_string(s), 0, qmax[s], kOverloadPenalty/o.scenarios, SCIP_VARTYPE_CONTINUOUS); vars.push_back(z[qid(k,i,s)]);
        }
      }
      for (int i = 0; i < N; ++i) for (int j = 0; j < N; ++j) if (i != j) {
        x[xid(k,i,j)] = AddVar(scip, "x_"+std::to_string(k)+"_"+std::to_string(i)+"_"+std::to_string(j), 0, 1, GetDistance(p,i,j), SCIP_VARTYPE_BINARY); vars.push_back(x[xid(k,i,j)]);
      }
    }

    for (int i = 1; i < N; ++i) {
      std::vector<std::pair<SCIP_VAR*,double>> a;
      for (int k = 0; k < K; ++k) a.push_back({y[yid(k,i)],1});
      AddLinear(scip,"assign_"+std::to_string(i),a,1,1);
    }
    for (int k = 0; k < K; ++k) {
      std::vector<std::pair<SCIP_VAR*,double>> depOut{{use[k],-1}}, depIn{{use[k],-1}};
      for (int j=1;j<N;++j) { depOut.push_back({x[xid(k,0,j)],1}); depIn.push_back({x[xid(k,j,0)],1}); }
      AddLinear(scip,"depot_out_"+std::to_string(k),depOut,0,0);
      AddLinear(scip,"depot_in_"+std::to_string(k),depIn,0,0);
      for (int i=1;i<N;++i) {
        std::vector<std::pair<SCIP_VAR*,double>> out{{y[yid(k,i)],-1}}, in{{y[yid(k,i)],-1}};
        for(int j=0;j<N;++j) if(i!=j) { out.push_back({x[xid(k,i,j)],1}); in.push_back({x[xid(k,j,i)],1}); }
        AddLinear(scip,"out_"+std::to_string(k)+"_"+std::to_string(i),out,0,0);
        AddLinear(scip,"in_"+std::to_string(k)+"_"+std::to_string(i),in,0,0);
      }
      for(int r=0;r<R;++r) {
        int pu=2*r+1, de=pu+1;
        AddLinear(scip,"pair_"+std::to_string(k)+"_"+std::to_string(r),{{y[yid(k,pu)],1},{y[yid(k,de)],-1}},0,0);
        AddLinear(scip,"precedence_"+std::to_string(k)+"_"+std::to_string(r),{{t[yid(k,de)],1},{t[yid(k,pu)],-1}},p.task_service_times[pu],SCIPinfinity(scip));
      }
      for(int i=0;i<N;++i) for(int j=1;j<N;++j) if(i!=j) {
        const double ei = i==0 ? GetEarliestStartTime(p,0) : GetEarliestStartTime(p,i);
        const double si = p.task_service_times[i];
        const double M = std::max(0.0, (i==0 ? GetEarliestStartTime(p,0) : GetLatestStartTime(p,i)) + si + GetDistance(p,i,j) - GetEarliestStartTime(p,j));
        std::vector<std::pair<SCIP_VAR*,double>> a{{t[yid(k,j)],1},{x[xid(k,i,j)],-M}};
        if(i>0) a.push_back({t[yid(k,i)],-1});
        const double rhs = (i == 0 ? ei : 0.0) + si + GetDistance(p,i,j) - M;
        AddLinear(scip,"time_"+std::to_string(k)+"_"+std::to_string(i)+"_"+std::to_string(j),a,rhs,SCIPinfinity(scip));
      }
      for(int i=1;i<N;++i) {
        const double M=std::max(0.0,GetLatestStartTime(p,i)+p.task_service_times[i]+GetDistance(p,i,0)-GetLatestStartTime(p,0));
        AddLinear(scip,"return_"+std::to_string(k)+"_"+std::to_string(i),{{t[yid(k,i)],1},{x[xid(k,i,0)],M}},-SCIPinfinity(scip),GetLatestStartTime(p,0)+M-p.task_service_times[i]-GetDistance(p,i,0));
      }
      for(int s=0;s<o.scenarios;++s) for(int j=1;j<N;++j) {
        const double d=p.task_demands[j][s], M=qmax[s]+std::fabs(d);
        AddLinear(scip,"qvisit_"+std::to_string(k)+"_"+std::to_string(j)+"_"+std::to_string(s),{{q[qid(k,j,s)],1},{y[yid(k,j)],-qmax[s]}},-SCIPinfinity(scip),0);
        AddLinear(scip,"over_"+std::to_string(k)+"_"+std::to_string(j)+"_"+std::to_string(s),{{z[qid(k,j,s)],1},{q[qid(k,j,s)],-1}},-cap,SCIPinfinity(scip));
        AddLinear(scip,"qdepLo_"+std::to_string(k)+"_"+std::to_string(j)+"_"+std::to_string(s),{{q[qid(k,j,s)],1},{x[xid(k,0,j)],-M}},d-M,SCIPinfinity(scip));
        AddLinear(scip,"qdepHi_"+std::to_string(k)+"_"+std::to_string(j)+"_"+std::to_string(s),{{q[qid(k,j,s)],1},{x[xid(k,0,j)],M}},-SCIPinfinity(scip),d+M);
        for(int i=1;i<N;++i) if(i!=j) {
          AddLinear(scip,"qlo_"+std::to_string(k)+"_"+std::to_string(i)+"_"+std::to_string(j)+"_"+std::to_string(s),{{q[qid(k,j,s)],1},{q[qid(k,i,s)],-1},{x[xid(k,i,j)],-M}},d-M,SCIPinfinity(scip));
          AddLinear(scip,"qhi_"+std::to_string(k)+"_"+std::to_string(i)+"_"+std::to_string(j)+"_"+std::to_string(s),{{q[qid(k,j,s)],1},{q[qid(k,i,s)],-1},{x[xid(k,i,j)],M}},-SCIPinfinity(scip),d+M);
        }
      }
    }
    for(int k=0;k+1<K;++k) AddLinear(scip,"sym_"+std::to_string(k),{{use[k],1},{use[k+1],-1}},0,SCIPinfinity(scip));

    const auto warm_start_path = ResolveWarmStart(o, instance);
    if (warm_start_path) {
      std::string rejection_reason;
      double warm_objective = SCIP_INVALID;
      const bool accepted = AddBksWarmStart(scip, p, *warm_start_path, R, N, K, o.scenarios, cap, qmax,
                                            x, y, t, q, z, use, rejection_reason, warm_objective);
      std::cout << "warm_start=" << *warm_start_path << " accepted=" << (accepted ? "true" : "false");
      if (accepted) std::cout << " objective=" << std::setprecision(15) << warm_objective;
      else std::cout << " reason=\"" << rejection_reason << '"';
      std::cout << '\n';
      if (!accepted && o.warm_start != "auto")
          throw std::runtime_error("explicit warm start was rejected: " + rejection_reason);
    } else {
      std::cout << "warm_start=none accepted=false reason=\"disabled or no matching BKS file\"\n";
    }

    std::cout << "instance=" << instance << " requests=" << R << " scenarios=" << o.scenarios
              << " routes=" << K << " capacity=" << cap << " penalty=" << kOverloadPenalty << '\n';
    Check(SCIPsolve(scip), "solve");
    // Releasing transformed variables / freeing SCIP can emit bookkeeping gap
    // events whose temporary bounds no longer describe the solved problem.
    // Freeze the event stream before writing the authoritative terminal row.
    trace.accept_events = false;
    WriteTrace(scip, &trace, "final", true);
    std::cout << std::setprecision(15) << "status=" << SCIPgetStatus(scip) << " incumbent=" << SCIPgetPrimalbound(scip)
              << " dual_bound=" << SCIPgetDualbound(scip) << " proof_gap=" << SCIPgetGap(scip)
              << " time_s=" << SCIPgetSolvingTime(scip) << " trace=" << o.trace << '\n';
    for (SCIP_VAR*& v : vars) Check(SCIPreleaseVar(scip, &v), "release variable");
    Check(SCIPfree(&scip), "SCIPfree");
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << '\n';
    if (scip) SCIPfree(&scip);
    return 2;
  }
}
