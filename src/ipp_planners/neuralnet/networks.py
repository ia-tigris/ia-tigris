from constants import *
import torch


class LinearModel(torch.nn.Module):
    def __init__(self, num_targets=10, num_particles=200, state_vector_size=4):
        super().__init__()
        num_targets = MAX_NUM_TRACKERS
        num_particles = NUM_PARTICLES
        state_vector_size = 4  # 8000
        drone_state_input_size = 5  # x y psi budget variance
        input_size = num_targets * num_particles * state_vector_size + drone_state_input_size


        self.layer_a  = torch.nn.Linear(input_size, 6000)
        self.layer_ab = torch.nn.BatchNorm1d(6000)
        self.layer_ar = torch.nn.ReLU()
        self.layer_ad = torch.nn.Dropout(0.5)

        self.layer_b  = torch.nn.Linear(6000, 6000)
        self.layer_bb = torch.nn.BatchNorm1d(6000)
        self.layer_br = torch.nn.ReLU()
        self.layer_bd = torch.nn.Dropout(0.5)

        self.layer_c  = torch.nn.Linear(6000, 4000)
        self.layer_cb = torch.nn.BatchNorm1d(4000)
        self.layer_cr = torch.nn.ReLU()
        self.layer_cd = torch.nn.Dropout(0.5)

        self.layer_d  = torch.nn.Linear(4000, 4000)
        self.layer_db = torch.nn.BatchNorm1d(4000)
        self.layer_dr = torch.nn.ReLU()
        self.layer_dd = torch.nn.Dropout(0.5)

        self.layer_e  = torch.nn.Linear(4000, 2000)
        self.layer_eb = torch.nn.BatchNorm1d(2000)
        self.layer_er = torch.nn.ReLU()
        self.layer_ed = torch.nn.Dropout(0.5)

        self.layer_f  = torch.nn.Linear(2000, 2000)
        self.layer_fb = torch.nn.BatchNorm1d(2000)
        self.layer_fr = torch.nn.ReLU()
        self.layer_fd = torch.nn.Dropout(0.5)

        self.layer_g  = torch.nn.Linear(2000, 1000)
        self.layer_gb = torch.nn.BatchNorm1d(1000)
        self.layer_gr = torch.nn.ReLU()
        self.layer_gd = torch.nn.Dropout(0.5)

        self.layer_h  = torch.nn.Linear(1000, 1000)
        self.layer_hb = torch.nn.BatchNorm1d(1000)
        self.layer_hr = torch.nn.ReLU()
        self.layer_hd = torch.nn.Dropout(0.5)

        self.layer_i  = torch.nn.Linear(1000, 500)
        self.layer_ib = torch.nn.BatchNorm1d(500)
        self.layer_ir = torch.nn.ReLU()
        self.layer_id = torch.nn.Dropout(0.5)

        self.layer_j  = torch.nn.Linear(500, 500)
        self.layer_jb = torch.nn.BatchNorm1d(500)
        self.layer_jr = torch.nn.ReLU()
        self.layer_jd = torch.nn.Dropout(0.5)

        self.layer_k  = torch.nn.Linear(500, 250)
        self.layer_kb = torch.nn.BatchNorm1d(250)
        self.layer_kr = torch.nn.ReLU()
        self.layer_kd = torch.nn.Dropout(0.5)

        self.layer_l  = torch.nn.Linear(250, 250)
        self.layer_lb = torch.nn.BatchNorm1d(250)
        self.layer_lr = torch.nn.ReLU()
        self.layer_ld = torch.nn.Dropout(0.5)

        self.layer_m  = torch.nn.Linear(250, 100)
        self.layer_mb = torch.nn.BatchNorm1d(100)
        self.layer_mr = torch.nn.ReLU()
        self.layer_md = torch.nn.Dropout(0.5)

        self.layer_n  = torch.nn.Linear(100, 1)

    def forward(self, drone_state, particle_state):

        # encode drone state to a vector of 100

        inp = torch.cat((drone_state, particle_state), dim=-1)

        a = self.layer_a(inp)  # 6000
        a = self.layer_ab(a)
        a = self.layer_ar(a)
        a = self.layer_ad(a)
        b = self.layer_b(a)  # 6000
        b = self.layer_bb(b)
        b = self.layer_br(b)
        b = self.layer_bd(b)
        c = self.layer_c(b + a)  # 4000  # residual connection
        c = self.layer_cb(c)
        c = self.layer_cr(c)
        c = self.layer_cd(c)
        d = self.layer_d(c)  # 4000
        d = self.layer_db(d)
        d = self.layer_dr(d)
        d = self.layer_dd(d)
        e = self.layer_e(d + c)  # 2000
        e = self.layer_eb(e)
        e = self.layer_er(e)
        e = self.layer_ed(e)
        f = self.layer_f(e)  # 2000
        f = self.layer_fb(f)
        f = self.layer_fr(f)
        f = self.layer_fd(f)
        g = self.layer_g(f + e)  # 1000
        g = self.layer_gb(g)
        g = self.layer_gr(g)
        g = self.layer_gd(g)
        h = self.layer_h(g)  # 1000
        h = self.layer_hb(h)
        h = self.layer_hr(h)
        h = self.layer_hd(h)
        i = self.layer_i(h + g)  # 500
        i = self.layer_ib(i)
        i = self.layer_ir(i)
        i = self.layer_id(i)
        j = self.layer_j(i)  # 500
        j = self.layer_jb(j)
        j = self.layer_jr(j)
        j = self.layer_jd(j)
        k = self.layer_k(j + i)  # 250
        k = self.layer_kb(k)
        k = self.layer_kr(k)
        k = self.layer_kd(k)
        l = self.layer_l(k)  # 250
        l = self.layer_lb(l)
        l = self.layer_lr(l)
        l = self.layer_ld(l)
        m = self.layer_m(l)  # 100
        m = self.layer_mb(m)
        m = self.layer_mr(m)
        m = self.layer_md(m)
        n = self.layer_n(m)  # 1

        return n
