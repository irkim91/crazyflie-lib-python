import numpy as np
from scipy.integrate import cumtrapz
import matplotlib.pyplot as plt

# 파라미터 설정
A = 0.5  # 진폭 (in)
omega_min = 2  # 최소 주파수 (r/s)
omega_max = 15   # 최대 주파수 (r/s)
C1 = 4.0
C2 = 0.0187
T_max = 2 * np.pi / omega_min
T_min = 2 * np.pi / omega_max
T_rec = 5 * T_max
dt = 0.01  # 시간 스텝 (s)
fade_duration = 5  # fade in/out 시간 (s)
trim_duration = 3  # 트림 명령 지속 시간 (s)

# 시간 벡터 생성
t = np.arange(0, T_rec + dt, dt)  # 총 시간 벡터

# 주파수 진행 계산
K = C2 * (np.exp(C1 * t / T_rec) - 1)
omega = omega_min + K * (omega_max - omega_min)

# theta(t) 계산
theta = cumtrapz(omega, t, initial=0)  # 누적 적분하여 theta 계산

# (delta_LON)sweep 신호 생성
delta_LON_sweep = A * np.sin(theta)

# Fade in/out 적용
fade_in_length = round(fade_duration / dt)  # fade in 길이 (samples)
fade_out_length = round(fade_duration / dt)  # fade out 길이 (samples)

# Fade in 가중치 생성
fade_in = np.linspace(0, 1, fade_in_length)  # 0에서 1로 증가

# Fade out 가중치 생성
fade_out = np.linspace(1, 0, fade_out_length)  # 1에서 0으로 감소

# 중간의 일정한 가중치
constant = np.ones(len(t) - fade_in_length - fade_out_length)

# 전체 가중치 생성
weight = np.concatenate((fade_in, constant, fade_out))

# 신호에 가중치 적용
delta_LON_sweep_faded = delta_LON_sweep * weight

# 트림 명령 생성 (3초간 0 값)
trim_length = round(trim_duration / dt)  # 트림 명령 길이 (samples)
trim_signal = np.zeros(trim_length)  # 트림 신호

# 전체 신호 생성 (트림 명령 + 페이드 적용 신호 + 트림 명령)
total_signal = np.concatenate((trim_signal, delta_LON_sweep, trim_signal, delta_LON_sweep, trim_signal))
total_signal_faded = np.concatenate((trim_signal, delta_LON_sweep_faded, trim_signal, delta_LON_sweep_faded, trim_signal))

# 결과 시각화
plt.figure(figsize=(10, 6))
plt.plot(np.arange(0, len(total_signal)) * dt, total_signal, label='Total Signal')
plt.plot(np.arange(0, len(total_signal_faded)) * dt, total_signal_faded, label='Total Signal Faded')
plt.xlabel('Time (s)')
plt.ylabel('Amplitude')
plt.title('Signal with Fade In/Out and Trim Commands')
plt.legend()
plt.grid()
plt.show()
