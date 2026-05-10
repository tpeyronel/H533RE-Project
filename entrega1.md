# Control de tracción para un auto a control remoto

## Resumen

El proyecto consistirá de un pequeño auto a control remoto que implementará un sistema de control de tracción (TC). Con un sistema de control de tracción, cuando una rueda con tracción patina, la computadora detecta este evento y reduce la fuerza del motor hacia esa rueda para evitar el deslizamiento excesivo, mejorando así el agarre. El auto no tendrá dirección, irá en linea recta. Se manejará con un joystick mediante comandos bluetooth. El joystick estará conectado a un software de PC, el cual mandará los correspondientes mensajes al auto mediante bluetooth. Dos ruedas tendrán tracción (directa), las otras dos serán libres. La detección de patinaje se hará mediante la utilización de tres encoders incrementales ópticos: uno en cada rueda con tracción, y otro en una de las dos ruedas libres.

## Objetivos

### Objetivo general

Implementar un sistema de control de tracción para mejorar la aceleración de un auto a control remoto ante una situación de patinaje. En general, la mejor aceleración posible se obtiene cuando las ruedas experimentan un poco de deslizamiento (entre 5% y 10%). Es decir, se busca implementar un deslizamiento controlado de las ruedas.

### Objetivos específicos
- Medir las velocidades de las distintas ruedas utilizando encoders incrementales.
- Detectar patinaje de las ruedas utilizando los datos de los encoders.
- Ajustar la potencia de los motores en base a la aceleración deseada por el usuario (i.e., que tanto intenta acelera con el joystick), limitada por el control de tracción cuando sea necesario para evitar patinaje. La potencia de los motores se regula mediante PWM. Esta parte requerirá de un controlador PID (o parecido) para lograr el deslizamiento óptimo de las ruedas.
- Implementar un protocol de comunicación bluetooth para los comandos de control del auto.

## Descripción funcional del sistema

### Diagrama de bloques

![diagrama de bloques](./diagrama.png)

### Entradas y salidas

#### Entradas
- Sensores: 
    - Tres encoders incrementales ópticos.
- Interfaz de usuario:
    - Un joystick para control de aceleración, entre otros.
- Señales de red:
    - Comandos bluetooth que se originan en una PC.

#### Salidas
- Motores: dos motores DC. Se controlará la potencia de estos mediante dos señales PWM.
- Posiblemente uno o más LEDs sobre el auto que indiquen distintas condiciones, como detección de deslizamiento, control de tracción activado, etc.
- Posiblemente el auto envíe información mediante bluetooth para visualizarla en la PC.

## Requisitos de tiempo real

La detección del deslizamiento de las ruedas (y el ajuste de potencia de los motores) requiere de tiempos de procesamiento muy cortos y consistentes. Esta sería la sección critica. Por otro lado, un apartado no tan importante con respecto a los tiempos sería la comunicación bluetooth. 

## Arquitectura de Software y diseño en FreeRTOS

### Tareas:
- Tarea de control de tracción: se encarga de detectar el deslizamiento, y de implementar el control PID para la potencia del motor (quizás esta tarea se separe en dos o más).
- Tarea de comunicación bluetooth: se encargar de recibir comandos de control de la PC, y potencialmente de enviar información de vuelta a la PC para visualización.

### Sincronización y comunicación
- Comunicación entre tareas:
    - La tarea de comunicación bluetooth se comunicará con la tarea de control de tracción utilizando variables compartidas. La tarea de control de tracción le enviará información del estado del sistema a la tarea de comunicación bluetooth mediante una queue.

- Manejo de interrupciones (ISRs): 
    - Se configurará un timer que generará interrupciones cada cierto tiempo para despertar la tarea de control de tracción.
    - Se generarán interrupciones cuando se reciban datos mediante bluetooth (UART).

## Arquitectura de Hardware
- Hardware: NUCLEO-H533RE
- Módulo bluetooth HC-06
- Driver motores DC
- Timers:
    - 3 timers para los encoders incrementales de las ruedas, configurados en modo de cuadratura.
    - 1 timer que define la frecuencia en la que se ejecuta la tarea de detección de deslizamiento.
    - 1 UART para la comunicación bluetooth serial.

## Metodología de validación
- Idealmente, se debería poder apreciar con el ojo humano que el auto acelera más rapido cuando el sistema de contol de tracción se encuentra activado.


