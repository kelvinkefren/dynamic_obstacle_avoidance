#!/usr/bin/env python3
import math
import numpy as np
import rospy
from std_msgs.msg import Bool, Float64
from dynamic_obstacle_avoidance.msg import CustomInfo

EPS = 1e-9

def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi

def norm(v: np.ndarray) -> float:
    return float(np.linalg.norm(v))

def unit(v: np.ndarray) -> np.ndarray:
    n = norm(v)
    if n < EPS:
        return np.zeros_like(v, dtype=float)
    return v / n

def rot_ccw(v: np.ndarray) -> np.ndarray:
    return np.array([-v[1], v[0]], dtype=float)

def rot_cw(v: np.ndarray) -> np.ndarray:
    return np.array([v[1], -v[0]], dtype=float)

def cross2(a: np.ndarray, b: np.ndarray) -> float:
    return float(a[0]*b[1] - a[1]*b[0])

def angle(v: np.ndarray) -> float:
    return math.atan2(float(v[1]), float(v[0]))

def theta_m_lyu(d: float, dm: float) -> float:
    # mesma forma usada no apf_lyu_2018.py (equivalente à arcsin(dm/d))
    if d <= dm + 1e-12:
        return math.pi/2.0
    return math.atan2(dm, math.sqrt(max(d*d - dm*dm, 1e-12)))

class ObstacleAvoidance:
    """
    Harmonic closed-form APF with:
      - NZ: Laplace in annulus (dm, CR), modes 0+1 (plus optional sin mode 1 for side bias)
      - EZ: Laplace in thin annulus (dm, dm+rho_E), modes 0+2, scaled by ||Vto||^2 (VTO only here)
    Returns the same tuple as your current code:
      total_force, attractive_force, repulsive_force, Frd_total, Frs_total, Fre_total
    """

    def __init__(self):
        # Mantém nomes compatíveis com teu projeto
        self.safe_distance = float(rospy.get_param('~safe_distance', 10.0))
        self.robot_domain_radius = float(rospy.get_param('~robot_domain_radius', 1.4))
        self.safety_margin_radius = float(rospy.get_param('~safety_margin_radius', 0.6))

        # rho_0 do Lyu (alcance do obstáculo)
        self.obstacle_influence_range = float(rospy.get_param('~obstacle_influence_range', 3.0*self.safe_distance))

        # Anel de emergência (quanto menor, mais "duro")
        self.rho_E = float(rospy.get_param('~emergency_band', 0.5))  # metros (ex.: 0.5)

        # Coeficientes NZ (poucos modos)
        # g_NZ = c0 + A1 cos(alpha) + B1 sin(alpha)
        self.nz_c0 = float(rospy.get_param('~nz_c0', 2000.0))
        self.nz_A1 = float(rospy.get_param('~nz_A1', 2000.0))
        self.nz_B1 = float(rospy.get_param('~nz_B1', 0.0))  # bias de lado (0 = sem bias)

        # Coeficientes EZ (poucos modos), multiplicados por ||Vto||^2
        # g_EZ = ||Vto||^2 * (c0E + A2 cos(2a) + B2 sin(2a))
        self.ez_c0 = float(rospy.get_param('~ez_c0', 100.0))
        self.ez_A2 = float(rospy.get_param('~ez_A2', 100.0))
        self.ez_B2 = float(rospy.get_param('~ez_B2', 0.0))

        # Atração simples (mantém tua saída de distância para o apf_force_to_vw_ref.py)
        self.att_gain = float(rospy.get_param('~att_gain', 3000.0))

        # Política de lado
        self.side_policy = str(rospy.get_param('~side_policy', 'starboard'))  # 'starboard' ou 'cross'

        # Pubs compatíveis com teu pipeline
        self.custom_info_pub = rospy.Publisher('/obstacle_avoidance/custom_info', CustomInfo, queue_size=10)
        self.distance_to_goal_pub = rospy.Publisher('/obstacle_avoidance/distance_to_goal', Float64, queue_size=10)
        self.collision_pub = rospy.Publisher('/obstacle_avoidance/collision', Bool, queue_size=10)

    def _choose_side_sign(self, p_ot: np.ndarray, v_to: np.ndarray, heading_dir: np.ndarray) -> float:
        """
        Retorna s em {+1,-1} para bias no termo tangencial.
        +1 = favorece e_theta (CCW do e_r em torno do obstáculo->OS)
        """
        # e_r aqui é obstáculo->OS = -(OS->obstáculo) = -unit(p_ot)
        e_r = unit(-p_ot)
        e_th = rot_ccw(e_r)

        if self.side_policy == 'cross':
            # sinal baseado no cross(p_ot, v_to) (mesma heurística do Lyu no "lado") 
            s = 1.0 if cross2(p_ot, v_to) >= 0.0 else -1.0
            return s

        # default: starboard (estilo Lyu)
        starboard = unit(rot_cw(heading_dir))
        cand1 = unit(rot_cw(unit(p_ot)))   # perp do n_ot (OS->TS)
        cand2 = unit(rot_ccw(unit(p_ot)))
        n_perp = cand1 if float(np.dot(cand1, starboard)) >= float(np.dot(cand2, starboard)) else cand2
        # converter para sinal em torno do obstáculo (e_theta baseado em obst->OS)
        # e_theta (obst->OS) = rot_ccw(-n_ot) = rot_cw(n_ot)
        # então dot com e_th dá o sinal correto
        s = 1.0 if float(np.dot(n_perp, e_th)) >= 0.0 else -1.0
        return s

    def _grad_nz(self, r: float, alpha: float, dm: float, CR: float, s: float):
        """
        Gradiente fechado do potencial NZ (modo 0 + 1):
          phi = c0 ln(b/r)/ln(b/a) + R1(r)[A1 cos a + B1 sin a]
        com a=dm, b=CR
        Retorna (dphi_dr, (1/r)dphi_dalpha)
        """
        a = max(dm, 1e-6)
        b = max(CR, a + 1e-6)
        r = float(np.clip(r, a + 1e-6, b - 1e-6))

        ln = math.log(b/a)
        D1 = a - (b*b)/a
        R1 = (r - (b*b)/r) / D1
        dR1 = (1.0 + (b*b)/(r*r)) / D1

        c0 = self.nz_c0
        A1 = self.nz_A1
        B1 = s * self.nz_B1

        base = (A1*math.cos(alpha) + B1*math.sin(alpha))

        dphi_dr = -c0/(r*ln) + dR1*base
        v_alpha = (R1/r)*(-A1*math.sin(alpha) + B1*math.cos(alpha))  # (1/r)*dphi/dalpha
        return dphi_dr, v_alpha

    def _grad_ez(self, r: float, alpha: float, dm: float, s: float, vto2: float):
        """
        Gradiente fechado do potencial EZ (modo 0 + 2), com VTO apenas aqui:
          phi = vto2*[ c0 ln(b/r)/ln(b/a) + R2(r)(A2 cos2a + B2 sin2a) ]
        anel: a=dm, b=dm+rho_E
        Retorna (dphi_dr, (1/r)dphi_dalpha)
        """
        a = max(dm, 1e-6)
        b = max(dm + max(self.rho_E, 1e-3), a + 1e-3)
        # se r<=dm, clampa para dentro do anel (barreira)
        r = float(np.clip(r, a + 1e-6, b - 1e-6))

        ln = math.log(b/a)
        D2 = a*a - (b**4)/(a*a)
        R2 = (r*r - (b**4)/(r*r)) / D2
        dR2 = (2.0*r + 2.0*(b**4)/(r**3)) / D2

        c0 = self.ez_c0
        A2 = self.ez_A2
        B2 = s * self.ez_B2

        c2 = math.cos(2.0*alpha)
        s2 = math.sin(2.0*alpha)
        base = (A2*c2 + B2*s2)

        dphi_dr = vto2 * (-c0/(r*ln) + dR2*base)
        v_alpha = vto2 * ((2.0*R2/r)*(-A2*s2 + B2*c2))  # (1/r)dphi/dalpha
        return dphi_dr, v_alpha

    def modified_potential_field(self, goal_position, obs_pos_list, obs_rad_list, obs_vel_list,
                                 os_pos, os_vel):
        # Goal
        p_os = np.array(os_pos, dtype=float)
        v_os = np.array(os_vel, dtype=float)
        p_g  = np.array(goal_position, dtype=float)

        p_og = p_g - p_os
        rho_og = norm(p_og)
        n_og = unit(p_og)

        # Publica distância (usada pelo apf_force_to_vw_ref.py)
        self.distance_to_goal_pub.publish(Float64(data=float(rho_og)))

        # Atração (mantém comportamento “forte” e simples)
        F_att = self.att_gain * rho_og * n_og

        F_rep = np.zeros(2, dtype=float)
        Frd_total = np.zeros(2, dtype=float)
        Fre_total = np.zeros(2, dtype=float)
        Frs_total = np.zeros(2, dtype=float)  # não usado (estático = dinâmico no teu requisito)

        # Heading_dir para política starboard
        if norm(v_os) > 1e-6:
            heading_dir = unit(v_os)
        else:
            heading_dir = n_og if rho_og > 1e-6 else np.array([1.0, 0.0], dtype=float)

        # Tau de colisão (igual tua lógica)
        tau = self.safety_margin_radius + self.robot_domain_radius

        for i, (p_ts_xy, R_ts, v_ts_xy) in enumerate(zip(obs_pos_list, obs_rad_list, obs_vel_list)):
            p_ts = np.array(p_ts_xy, dtype=float)
            v_ts = np.array(v_ts_xy, dtype=float)
            R_ts = float(R_ts)

            p_ot = p_ts - p_os          # OS -> TS (como Lyu)
            d = norm(p_ot)

            # colisão “geométrica”
            col = Bool(data=(d < tau))
            self.collision_pub.publish(col)

            dm = self.robot_domain_radius + self.safe_distance + R_ts
            CR = dm + self.obstacle_influence_range

            # fora do alcance
            if d > CR:
                continue

            v_to = v_os - v_ts
            vto2 = float(np.dot(v_to, v_to))

            # ângulos do cone
            # alpha = angle between p_ot and v_to (via difference of headings)
            if norm(v_to) > 1e-6:
                alpha = wrap_pi(angle(p_ot) - angle(v_to))
            else:
                alpha = math.pi  # sem movimento relativo -> não ativa cone

            th = abs(alpha)
            thm = theta_m_lyu(d, dm)

            # vetores unitários (obstáculo->OS) para força
            e_r = unit(p_os - p_ts)     # obstáculo -> OS
            e_th = rot_ccw(e_r)

            # sinal de lado
            s = self._choose_side_sign(p_ot, v_to, heading_dir)

            ci = CustomInfo()
            ci.distance_to_obstacle = float(d)
            ci.center_to_center_safe_distance = float(dm)
            ci.collision_avoidance_radius = float(CR)
            ci.vector_to_obstacle = p_ot.tolist()
            ci.relative_speed_vector = v_to.tolist()
            ci.unit_vector_to_obstacle = unit(p_ot).tolist() if d > 1e-6 else [0.0, 0.0]
            ci.perpendicular_unit_vector_to_obstacle = (s * e_th).tolist()
            ci.obstacle_domain_radius = float(R_ts)
            ci.distance_to_goal = float(rho_og)
            ci.dm = float(dm)
            ci.CR = float(CR)
            ci.angle_between_direction_and_velocity = float(math.degrees(th))
            ci.angle_for_safe_distance = float(math.degrees(thm))
            ci.angle_difference_for_safety = float(math.degrees(thm - th))

            # --- EZ: d <= dm  (VTO SÓ AQUI) ---
            if d <= dm:
                ci.action_type = "Obstáculo Emergência (Harmonic EZ)"
                ci.avoidance_type = "HarmonicClosed EZ (VTO only)"

                dphi_dr, v_alpha = self._grad_ez(d, alpha, dm, s, vto2)
                # F = -grad(phi)
                F = -(dphi_dr * e_r + v_alpha * e_th)
                Fre_total += F
                F_rep += F
                self.custom_info_pub.publish(ci)
                continue

            # --- NZ: dm < d <= CR and theta < theta_m ---
            if th < thm:
                ci.action_type = "Obstáculo NZ (Harmonic NZ)"
                ci.avoidance_type = "HarmonicClosed NZ (few modes)"

                dphi_dr, v_alpha = self._grad_nz(d, alpha, dm, CR, s)
                F = -(dphi_dr * e_r + v_alpha * e_th)
                Frd_total += F
                F_rep += F
                self.custom_info_pub.publish(ci)

        F_total = F_att + F_rep
        return F_total, F_att, F_rep, Frd_total, Frs_total, Fre_total

