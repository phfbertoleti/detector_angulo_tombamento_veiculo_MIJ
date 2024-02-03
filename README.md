# Projeto - detector de ângulo de tombamento com o Módulo IoT JVTECH (MIJ)

Neste repositório, você encontra o projeto do detector de ângulo de tombamento de veículos, feito para o Módulo IoT JVTECH (MIJ).
Este exemplo tem as seguintes funcionalidades:

* Ler periodicamente a IMU BMI270 (via I²C) e obter as medições de acelerômetro e giroscópio (nos eixos X, Y e Z)
* Calcular os ângulos de Euler (roll, pitch e yaw) via algoritmo Mahony AHRS e obter o ângulo de tombamento (equivalente ao ângulo de roll)
* Periodicamente enviar, via LoRa, o ângulo de tombamento calculado. O envio é feito via mensagem encapsulada no seguinte protocolo: preâmbulo "ANG", ângulo de tombamento (formato: float) e checksum de 1 byte