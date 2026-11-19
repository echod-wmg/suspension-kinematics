"""Bridge to the original 2020 Excel tool.

The spreadsheet was driven by a handful of *parametric* inputs (chassis
widths/heights, arm lengths, knuckle length, tyre size) rather than 3-D
hardpoints. :class:`ExcelDoubleWishbone` accepts exactly those inputs and
turns them into a :class:`~suspkin.models.DoubleWishbone` so old models can
be re-run with the exact solver.

It also reproduces the spreadsheet's own (approximate) camber calculation in
:func:`ExcelDoubleWishbone.excel_camber`, which is used in
``validation/`` to show how the approximation compares with ADAMS and with
the exact loop closure.

Notes on the spreadsheet conventions that are reproduced here:

* "Tyre height" was computed as ``rim_diameter + one sidewall``
  (``R*25.4 + W*AR/100``) and then used as the *overall* wheel diameter.
  That under-sizes the wheel by one sidewall; it is kept by default
  (``legacy_tyre=True``) so the geometry matches the 2020 models and the
  ADAMS validation model built from them.
* The knuckle is centred on the wheel centre and the lower ball joint sits
  ``knuckle_length / 2`` below it.
* Caster was applied by moving the upper wishbone pivots fore/aft.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass

import numpy as np

from .geometry import line_intersection_2d
from .models import DoubleWishbone, Wheel


@dataclass
class ExcelDoubleWishbone:
    chassis_upper_width: float
    chassis_lower_width: float
    chassis_lower_height: float
    chassis_upper_height: float
    upper_arm_length: float
    lower_arm_length: float
    knuckle_length: float
    rim_diameter_in: float
    tyre_width: float
    tyre_aspect_ratio: float
    upper_arm_width: float = 300.0
    lower_arm_width: float = 350.0
    caster_deg: float = 0.0
    toe_deg: float = 0.0
    steering_arm: float = 120.0
    legacy_tyre: bool = True

    # -- geometry --------------------------------------------------------------
    @property
    def tyre_height(self) -> float:
        sidewall = self.tyre_width * self.tyre_aspect_ratio / 100.0
        rim = self.rim_diameter_in * 25.4
        return rim + sidewall if self.legacy_tyre else rim + 2 * sidewall

    @property
    def wheel_radius(self) -> float:
        return self.tyre_height / 2.0

    def _pivots(self):
        yl, zl = self.chassis_lower_width / 2, self.chassis_lower_height
        yu, zu = self.chassis_upper_width / 2, self.chassis_upper_height
        return yl, zl, yu, zu

    def hardpoints(self) -> dict:
        yl, zl, yu, zu = self._pivots()
        k = self.knuckle_length
        c = np.radians(self.caster_deg)
        z_lbj = self.wheel_radius - k / 2
        y_lbj = yl + np.sqrt(self.lower_arm_length**2 - (zl - z_lbj) ** 2)
        x_ubj, z_ubj = -k * np.sin(c), z_lbj + k * np.cos(c)
        y_ubj = yu + np.sqrt(self.upper_arm_length**2 - (z_ubj - zu) ** 2)
        lbj = np.array([0.0, y_lbj, z_lbj])
        ubj = np.array([x_ubj, y_ubj, z_ubj])
        wc = 0.5 * (lbj + ubj)
        wc[2] = self.wheel_radius
        # Tie rod (the spreadsheet had none - toe was applied directly): the
        # steering arm sits behind the knuckle at wheel-centre height and the
        # inner joint is placed where the line from the outer joint to the
        # front-view instant centre crosses the line through the inner pivots.
        # That is the classic first-order zero bump-steer layout.
        x_tr = wc[0] - self.steering_arm
        z_tr = wc[2]
        y_tro = y_lbj + (y_ubj - y_lbj) * (z_tr - z_lbj) / (z_ubj - z_lbj) - 20.0
        ic = line_intersection_2d((yl, zl), (y_lbj - yl, z_lbj - zl), (yu, zu), (y_ubj - yu, z_ubj - zu))
        tri = None
        if ic is not None:
            tri = line_intersection_2d((y_tro, z_tr), ic - np.array([y_tro, z_tr]), (yl, zl), (yu - yl, zu - zl))
        if tri is None or not (min(zl, zu) - 50 <= tri[1] <= max(zl, zu) + 50):
            tri = np.array([yl + (yu - yl) * (z_tr - zl) / (zu - zl), z_tr])
        y_tri, z_tri = tri
        hu, hl = self.upper_arm_width / 2, self.lower_arm_width / 2
        return {
            "lower_front_pivot": [hl, yl, zl],
            "lower_rear_pivot": [-hl, yl, zl],
            "lower_ball_joint": lbj.tolist(),
            "upper_front_pivot": [x_ubj + hu, yu, zu],
            "upper_rear_pivot": [x_ubj - hu, yu, zu],
            "upper_ball_joint": ubj.tolist(),
            "tie_rod_inner": [x_tr, float(y_tri), float(z_tri)],
            "tie_rod_outer": [x_tr, y_tro, z_tr],
            "wheel_centre": wc.tolist(),
        }

    def static_camber(self) -> float:
        """The spreadsheet's wheel plane is parallel to the knuckle line."""
        h = self.hardpoints()
        dy = h["upper_ball_joint"][1] - h["lower_ball_joint"][1]
        dz = h["upper_ball_joint"][2] - h["lower_ball_joint"][2]
        return float(np.degrees(np.arctan2(dy, dz)))

    def to_model(self, travel_reference: str = "wheel_centre") -> DoubleWishbone:
        wheel = Wheel(
            radius=self.wheel_radius,
            width=self.tyre_width,
            static_camber_deg=self.static_camber(),
            static_toe_deg=self.toe_deg,
        )
        return DoubleWishbone(self.hardpoints(), wheel, travel_reference)

    def to_dict(self) -> dict:
        return asdict(self)

    # -- the original spreadsheet approximation ------------------------------
    def excel_camber(self, travel):
        """Camber (deg) exactly as the 2020 spreadsheet computed it.

        The spreadsheet moved the lower ball joint vertically by ``travel``,
        then placed the upper ball joint ``knuckle_length`` *directly above*
        it (i.e. it assumed the knuckle stays vertical) to get the upper arm
        angle, and finally took camber from the lateral offset between the two
        ball joints. The vertical-knuckle assumption is what makes the error
        grow at large travel. Positive = top of wheel outboard.
        """
        travel = np.asarray(travel, dtype=float)
        yl, zl, yu, zu = self._pivots()
        k = self.knuckle_length
        z_l = self.wheel_radius - k / 2 + travel
        y_l = yl + np.sqrt(self.lower_arm_length**2 - (zl - z_l) ** 2)
        z_u = z_l + k
        y_u = yu + np.sqrt(self.upper_arm_length**2 - (z_u - zu) ** 2)
        return np.degrees(np.arcsin((y_u - y_l) / k))
