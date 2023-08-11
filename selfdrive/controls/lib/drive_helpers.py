import math
import os

import threading
import time

from cereal import car, log, messaging
from common.conversions import Conversions as CV
from common.numpy_fast import clip, interp
from common.realtime import DT_MDL
from selfdrive.modeld.constants import T_IDXS

# WARNING: this value was determined based on the model's training distribution,
#          model predictions above this speed can be unpredictable
# V_CRUISE's are in kph
V_CRUISE_MIN = 8
V_CRUISE_MAX = 145
V_CRUISE_UNSET = 255
V_CRUISE_INITIAL = 40
V_CRUISE_INITIAL_EXPERIMENTAL_MODE = 105
IMPERIAL_INCREMENT = 1.6  # should be CV.MPH_TO_KPH, but this causes rounding errors

MIN_SPEED = 1.0
CONTROL_N = 17
CAR_ROTATION_RADIUS = 0.0

# EU guidelines
MAX_LATERAL_JERK = 5.0

MAX_VEL_ERR = 5.0

ButtonEvent = car.CarState.ButtonEvent
ButtonType = car.CarState.ButtonEvent.Type
CRUISE_LONG_PRESS = 50
CRUISE_NEAREST_FUNC = {
  ButtonType.accelCruise: math.ceil,
  ButtonType.decelCruise: math.floor,
}
CRUISE_INTERVAL_SIGN = {
  ButtonType.accelCruise: +1,
  ButtonType.decelCruise: -1,
}


class VCruiseHelper:
  def __init__(self, CP):
    self.CP = CP
    self.v_cruise_kph = V_CRUISE_UNSET
    self.v_cruise_cluster_kph = V_CRUISE_UNSET
    self.v_cruise_kph_last = 0
    self.button_timers = {ButtonType.decelCruise: 0, ButtonType.accelCruise: 0}
    self.button_change_states = {btn: {"standstill": False, "enabled": False} for btn in self.button_timers}

    #######################
    #OVERIDE INITIALIZATION
    #######################

    self.v_cruise_suggested = 0 # set by _get_suggested_speed
    self.sm = messaging.SubMaster(['liveLocationKalman', 'carState'])

    # log files
    self.dirname = os.path.dirname(__file__)
    self.speed_file = os.path.join(self.dirname, 'data/speed.txt')
    self.coordinates_file = os.path.join(self.dirname, 'data/coordinates.txt')

    # overwrite previous file
    with open(self.speed_file, 'w') as sf, open(self.coordinates_file, 'w') as cf:
      sf.write('')
      cf.write('')

    threading.Thread(target=self._get_suggested_speed, daemon=True).start()

  @property
  def v_cruise_initialized(self):
    return self.v_cruise_kph != V_CRUISE_UNSET

  def update_v_cruise(self, CS, enabled, is_metric):
    self.v_cruise_kph_last = self.v_cruise_kph

    if CS.cruiseState.available:
      if not self.CP.pcmCruise:
        # if stock cruise is completely disabled, then we can use our own set speed logic
        self._update_v_cruise_non_pcm(CS, enabled, is_metric)
        self.v_cruise_cluster_kph = self.v_cruise_kph
        self.update_button_timers(CS, enabled)
      else: # override pcmCruise to see suggested speed, change this to UI only in the future
        self._update_v_cruise_non_pcm(CS, enabled, is_metric)
        self.v_cruise_cluster_kph = self.v_cruise_kph
        # self.v_cruise_kph = CS.cruiseState.speed * CV.MS_TO_KPH
        # self.v_cruise_cluster_kph = CS.cruiseState.speedCluster * CV.MS_TO_KPH
    else:
      self.v_cruise_kph = V_CRUISE_UNSET
      self.v_cruise_cluster_kph = V_CRUISE_UNSET

  def _update_v_cruise_non_pcm(self, CS, enabled, is_metric):
    # handle button presses. TODO: this should be in state_control, but a decelCruise press
    # would have the effect of both enabling and changing speed is checked after the state transition
    if not enabled:
      return

    # long_press = False
    # button_type = None

    # v_cruise_delta = 1. if is_metric else IMPERIAL_INCREMENT

    # for b in CS.buttonEvents:
    #   if b.type.raw in self.button_timers and not b.pressed:
    #     if self.button_timers[b.type.raw] > CRUISE_LONG_PRESS:
    #       return  # end long press
    #     button_type = b.type.raw
    #     break
    # else:
    #   for k in self.button_timers.keys():
    #     if self.button_timers[k] and self.button_timers[k] % CRUISE_LONG_PRESS == 0:
    #       button_type = k
    #       long_press = True
    #       break

    # if button_type is None:
    #   return

    # # Don't adjust speed when pressing resume to exit standstill
    # cruise_standstill = self.button_change_states[button_type]["standstill"] or CS.cruiseState.standstill
    # if button_type == ButtonType.accelCruise and cruise_standstill:
    #   return

    # # Don't adjust speed if we've enabled since the button was depressed (some ports enable on rising edge)
    # if not self.button_change_states[button_type]["enabled"]:
    #   return

    # v_cruise_delta = v_cruise_delta * (5 if long_press else 1)
    # if long_press and self.v_cruise_kph % v_cruise_delta != 0:  # partial interval
    #   self.v_cruise_kph = CRUISE_NEAREST_FUNC[button_type](self.v_cruise_kph / v_cruise_delta) * v_cruise_delta
    # else:
    #   self.v_cruise_kph += v_cruise_delta * CRUISE_INTERVAL_SIGN[button_type]

    # # If set is pressed while overriding, clip cruise speed to minimum of vEgo
    # if CS.gasPressed and button_type in (ButtonType.decelCruise, ButtonType.setCruise):
    #   self.v_cruise_kph = max(self.v_cruise_kph, CS.vEgo * CV.MS_TO_KPH)

    # if self.v_cruise_kph < self.v_cruise_suggested:
    #   self.v_cruise_kph += v_cruise_delta
    # elif self.v_cruise_kph > self.v_cruise_suggested:
    #   self.v_cruise_kph -= v_cruise_delta

    self.v_cruise_kph = clip(round(self.v_cruise_suggested, 1), V_CRUISE_MIN, V_CRUISE_MAX)

  def update_button_timers(self, CS, enabled):
    # increment timer for buttons still pressed
    for k in self.button_timers:
      if self.button_timers[k] > 0:
        self.button_timers[k] += 1

    for b in CS.buttonEvents:
      if b.type.raw in self.button_timers:
        # Start/end timer and store current state on change of button pressed
        self.button_timers[b.type.raw] = 1 if b.pressed else 0
        self.button_change_states[b.type.raw] = {"standstill": CS.cruiseState.standstill, "enabled": enabled}

  def initialize_v_cruise(self, CS, experimental_mode: bool) -> None:
    # initializing is handled by the PCM
    if self.CP.pcmCruise:
      return

    initial = V_CRUISE_INITIAL_EXPERIMENTAL_MODE if experimental_mode else V_CRUISE_INITIAL

    # 250kph or above probably means we never had a set speed
    if any(b.type in (ButtonType.accelCruise, ButtonType.resumeCruise) for b in CS.buttonEvents) and self.v_cruise_kph_last < 250:
      self.v_cruise_kph = self.v_cruise_kph_last
    else:
      self.v_cruise_kph = int(round(clip(CS.vEgo * CV.MS_TO_KPH, initial, V_CRUISE_MAX)))

    self.v_cruise_cluster_kph = self.v_cruise_kph

  #################
  # THREADED CLIENT
  #################

  # def _get_suggested_speed(self) -> None:
  #   '''Sends GPS coordinates, receives suggested speed from server every second'''
  #   # HOST = "128.195.153.42" # the server's IP address (opal PC)
  #   # HOST = "192.168.153.27" # Eric's PC
  #   HOST = "172.20.10.6" # laptop IP on Blake's hotspot
  #   PORT = 65432
    
  #   with socket.socket(socket.AF_INET,socket.SOCK_STREAM) as s:
  #     s.connect((HOST,PORT))
  #     coordinate_str = '(-1,-1)'      
      
  #     while True:

  #       # this caused an error for some reason
  #       real_location = self.sm['liveLocationKalman']
  #       try:
  #         coordinate_str = real_location.positionGeodetic.value[0] + ', ' + real_location.positionGeodetic.value[1]
  #       except:
  #         pass
        
  #       # carla_location = self.sm['gpsLocationExternal']
  #       # coordinate_str = str(carla_location.latitude) + ' ,' + str(carla_location.longitude)
        
  #       s.sendall(bytes(coordinate_str, encoding='UTF-8'))
  #       data = s.recv(1024)
        
  #       # convert mph to kph
  #       self.v_cruise_suggested = round(float((data.decode("utf-8")))*CV.MPH_TO_KPH)
        
  #       # self.v_cruise_suggested = float(data.decode("utf-8"))
  #       time.sleep(1)


  ##########################
  # FILE WRITING FOR TESTING
  ##########################

  def _get_suggested_speed(self) -> None:

    increment = 10
    
    while True:
      self.sm.update()
      real_location, car_state = self.sm['liveLocationKalman'], self.sm['carState']

      coordinate_str = '(-1,-1)'
      current_speed = car_state.vEgo * CV.MS_TO_MPH
      cruise_speed = -1 
        
      if real_location.positionGeodetic.valid:
        coordinate_str = f'({real_location.positionGeodetic.value[0]}, {real_location.positionGeodetic.value[1]})'

      if car_state.cruiseState.available:
        cruise_speed = car_state.cruiseState.speed * CV.MS_TO_MPH

      suggested_speed = current_speed + increment # MPH

      with open(self.speed_file, 'a') as sf, open(self.coordinates_file, 'a') as cf:
        # write coordinates
        cf.write(f'{coordinate_str}\n')

        # get speed from file
        # self.v_cruise_suggested = round(float(sf.read())*CV.MPH_TO_KPH)

        # write to file: actual, set, suggested
        sf.write(f'{current_speed}, {cruise_speed}, {suggested_speed}\n')
      
      if suggested_speed >= 0:
        self.v_cruise_suggested = round(suggested_speed*CV.MPH_TO_KPH)
      
      increment *= -1
      
      time.sleep(20) 

def apply_deadzone(error, deadzone):
  if error > deadzone:
    error -= deadzone
  elif error < - deadzone:
    error += deadzone
  else:
    error = 0.
  return error


def apply_center_deadzone(error, deadzone):
  if (error > - deadzone) and (error < deadzone):
    error = 0.
  return error


def rate_limit(new_value, last_value, dw_step, up_step):
  return clip(new_value, last_value + dw_step, last_value + up_step)


def get_lag_adjusted_curvature(CP, v_ego, psis, curvatures, curvature_rates):
  if len(psis) != CONTROL_N:
    psis = [0.0]*CONTROL_N
    curvatures = [0.0]*CONTROL_N
    curvature_rates = [0.0]*CONTROL_N
  v_ego = max(MIN_SPEED, v_ego)

  # TODO this needs more thought, use .2s extra for now to estimate other delays
  delay = CP.steerActuatorDelay + .2

  # MPC can plan to turn the wheel and turn back before t_delay. This means
  # in high delay cases some corrections never even get commanded. So just use
  # psi to calculate a simple linearization of desired curvature
  current_curvature_desired = curvatures[0]
  psi = interp(delay, T_IDXS[:CONTROL_N], psis)
  average_curvature_desired = psi / (v_ego * delay)
  desired_curvature = 2 * average_curvature_desired - current_curvature_desired

  # This is the "desired rate of the setpoint" not an actual desired rate
  desired_curvature_rate = curvature_rates[0]
  max_curvature_rate = MAX_LATERAL_JERK / (v_ego**2) # inexact calculation, check https://github.com/commaai/openpilot/pull/24755
  safe_desired_curvature_rate = clip(desired_curvature_rate,
                                     -max_curvature_rate,
                                     max_curvature_rate)
  safe_desired_curvature = clip(desired_curvature,
                                current_curvature_desired - max_curvature_rate * DT_MDL,
                                current_curvature_desired + max_curvature_rate * DT_MDL)

  return safe_desired_curvature, safe_desired_curvature_rate


def get_friction(lateral_accel_error: float, lateral_accel_deadzone: float, friction_threshold: float, torque_params: car.CarParams.LateralTorqueTuning, friction_compensation: bool) -> float:
  friction_interp = interp(
    apply_center_deadzone(lateral_accel_error, lateral_accel_deadzone),
    [-friction_threshold, friction_threshold],
    [-torque_params.friction, torque_params.friction]
  )
  friction = float(friction_interp) if friction_compensation else 0.0
  return friction


def get_speed_error(modelV2: log.ModelDataV2, v_ego: float) -> float:
  # ToDo: Try relative error, and absolute speed
  if len(modelV2.temporalPose.trans):
    vel_err = clip(modelV2.temporalPose.trans[0] - v_ego, -MAX_VEL_ERR, MAX_VEL_ERR)
    return float(vel_err)
  return 0.0
