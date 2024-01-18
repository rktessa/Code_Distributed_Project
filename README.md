# Distributed_Systems
py -m venv .env
python3 -m venv .env

University Project
January 23
Riccardo Tessarin and Simone Carli

## Obiettivo del progetto
Creare una infrastruttura hardware per testare un algoritmo di distributed system. usando dei differential drive robots che si muovono autonomamente nello spazio. 

In particolare vi è l'implementazione di una rete di comunicazione totalmente distributa, sfruttando il protocollo ad Hoc e l'uso di inidizzi ip statici e la creazione di un sistema di comunicazione e messaggistica per la diffusione di informazioni tra gli agent nel network.

### Strumenti usati: 
- Batman-adv, un sistema per implementare reti ad hoc, che si trova già nel kernel di linux;
- ZeroMQ un ptotocollo di comunicazione veloce e leggero da implementare;
-  Piattaforma Arduino per il controllo dei robots e la lettura dei sensori installati nella scheda
- Raspberry per la comunicazione
- Algoritmi di obstacle avoidance e logica di contollo per far muovere i droni

## Algoritmo da testare

Localizzazione  usando ranging e IMU. Simulazione in matlab, con n punti (5 o 7), simulare un accelerometro e un uwb e implementare un multi dimensional scaling e un filtro gpb. 

L'idea è risolvere la mappa relativa togliendo le ambiguità di rotation e di flip usando queste due misure. 
Creazione di una mappa globale o relativa, in cui tutti sanno dove sono gli altri veramente. Ognuno deve implementare il filtro gpb, che partendo dal mds ( per trovare la geometria relativa) e dalle misure continua a cercare di capire in quale configurazione si trova
MDS = Multi dimensional scaling
GPB = Generalised Pseudo Bayesian, sono N filtri di kalman con modelli diversi, per capire se sono a v costante, quasi statico o uniformenente accelerato. Non so in quali di questi 3 stati sono, ma posso essere in uno solo  di questi. 
Il caso più generalizzato di questo modello è l'IMM: Interactive Multiple Model, ma a noi basta lo step precedente, non può succedere questo. 

Quasi tutto si fa su matlab, con la soluzione, si prendono n robottini e si può fare un esperimento. 
Una soluzione fatta così non c'è al momento. 

Convergere ad una soluzione unendo queste due misurazioni e usando il filtro gpb è la cosa interessante.

Ipotesi di quanti modelli vanno instanziati: 
- quello che esce dal mds
- la sua versione specchiata su un asse
- la sua versione specchiata nell'altro asse
- la rotazione la decido a priori perchè decido io a monte come è ruotato il sistema, "per me stesso la terna è ruotata così"

A priori non so se la soluzione del mds è corretta, devo verificarlo. Vedere se di questi 3 modelli uno ha il mode, la probabilità che va a 1. 

GPB 1 su github...
basta cambiare il modello

Idea finale, Swarming con piattaforma per navigare o esplorare un ambiente, la soluzione è agnostica da questo.

Navigazione, conosco la mappa
Esplorazione devo anche costruire la mappa. 

Swarm per il mantenimento di una certa formazione è interessante. 


 ### Link utili a video per imparare a implementare
- More details on dead reckoning, MATLAB Tech Talk video: https://bit.ly/37T9BRT
- Understanding the Kalman Filter, MATLAB Tech Talk Series: https://bit.ly/314rLia
- Another good description of the particle filter: https://youtu.be/aUkBa1zMKv4
- Download ebook: Sensor Fusion and Tracking for Autonomous Systems: An Overview - https://bit.ly/2YZxvXA
- Download white paper: Sensor Fusion and Tracking for Autonomous Systems - https://bit.ly/2YZxvXA
- A Tutorial on Particle Filtering and Smoothing (includes AMCL). Paper by Doucet and Johansen: https://www.stats.ox.ac.uk/~doucet/do...


Io ho un MDS implementato preso da qui dove è tutto spiegato molto bene anche:
==> https://nbviewer.org/github/drewwilimitis/Manifold-Learning/blob/master/Multidimensional_Scaling.ipynb

Altro articolo da guardare: https://ieeexplore.ieee.org/document/10008692

Questa pagina web è molto interessante e presenta in modo chiaro come funziona
MDS implementato in sklearn ==> https://stackabuse.com/guide-to-multidimensional-scaling-in-python-with-scikit-learn/


### Analisi e messa in ordine articoli trovati
- **Solving Ambiguities in MDS Relative Localization:** Presenta lavoro su come risolvere ste benedette ambiguità dovute alla localizzazione. 
        Questo articolo mi rimanda a **A. Kannan, B. Fidan, G. Mao, and B. Anderson, “Analysis of flip ambiguities in distributed network localization,”** che spiega bene le ambiguità
- **Generalized pseudo Bayesian algorithms for tracking of multiple model underwater maneuvering target** questo è molto interessnte per vedere come costruire i GPB 1 estimators. 


# Creazione Rete Mesh con Raspberry

Per creare la rete mesh con raspberry i seguenti passaggi sono stati necessari: 

1. Per prima cosa installare 
***
```sudo apt-get update && sudo apt-get upgrade -y``` 
***

2. Poi installare
***
```sudo apt-get install -y batctl``` 


Bisogna verificare che Batman sia caricato prima di cercare di usarlo
inserire questo codice nel file ```sudo nano /etc/modules```

```
# /etc/modules: kernel modules to load at boot time.
#
# This file contains the names of kernel modules that should be loaded
# at boot time, one per line. Lines beginning with "#" are ignored.

i2c-dev
batman-adv
```

3. Andare in ```cd /etc/network/interfaces.d```

4. Creare file sudo nano wlan0 e inserire le seguenti linee di codice:
***
```
auto wlan0
iface wlan0 inet manual
    mtu 1468
    wireless-channel 1
    wireless-essid RPi-mesh
    wireless-mode ad-hoc
    # wireless-ap 02:12:34:56:78:9A
```

Se è già presente un file **wlan0** commenta con un # le linee del codice presenti. 

Funziona anche con il comando ```iface wlan0 inet6 manual``` e impostando a 1532 valore del MTU. 

5. Allo stesso modo creare un secondo file sempre nella medesima posizione, chiamandolo **bat0**
***
```
auto bat0
#iface bat0 inet6 auto
#   pre-up /usr/sbin/batctl if add eth0
#   pre-up /usr/sbin/batctl if add wlan0
iface bat0 inet auto
   address 192.168.123.3
   netmask 255.255.255.0
   gateway 192.168.123.1
   pre-up /usr/sbin/batctl if add etho0
   pre-up /usr/sbin/batctl if add wlan0
```

Le prime tre righe sono commentate perchè in questa configurazione non sto usando il protocollo **ipv6** ma solo quello ipv4. 

### Nodo Mesh

6. Per un nodo di mesh va poi inserito il seguente codice all'interno del file ```sudo nano /etc/rc.local ```

```
#!/bin/sh -e
sudo batctl if add wlan0 &
sudo ifconfig bat0 mtu 1468 &
sudo batctl gw_mode client &
sudo ifconfig bat0 up &
sudo ifconfig wlan0 up &

exit 0
```
7. Interompere il  processo di DHCP per evitare che provi a gestire l'interfaccia wireless lan usando il seguente comando :
***
```
echo 'denyinterfaces wlan0' | sudo tee --append /etc/dhcpcd.conf
```

A questo punto al riavvio il node di mesh si collega automaticamente alla rete ad-hoc creata. 

### Nodo Gateway
1. Installare il seguente pacchetto per la gestione del software DHCP
```sudo apt-get install -y dnsmasq```

2. Configurare il server DHCP con le seguenti impostazioni: 
```sudo nano /etc/dnsmasq.conf```
```
# Da inserire in fondo a tutto il file commentato
interface=bat0
dhcp-range=192.168.123.2,192.168.123.99,255.255.255.0,12h
```


3. Per un nodo di Gateway va poi inserito il seguente codice all'interno del file ```sudo nano /etc/rc.local ```

```
#!/bin/sh -e
#
# rc.local
#
# This script is executed at the end of each multiuser runlevel.
# Make sure that the script will "exit 0" on success or any other
# value on error.
#
# In order to enable or disable this script just change the execution
# bits.
#
# By default this script  execute the sh file below


sudo batctl meshif bat0 if add wlan0 &
sudo ifconfig bat0 mtu 1468 &

sudo batctl gw_mode server &

sudo sysctl -w net.ipv4.ip_forward=1
sudo iptables -t nat -A POSTROUTING -o wlan1 -j MASQUERADE &
sudo iptables -A FORWARD -i wlan1 -o bat0 -m conntrack --ctstate RELATED,ESTABLISHED -j ACCEPT &
sudo iptables -A FORWARD -i bat0 -o wlan1 -j ACCEPT &



sudo ifconfig wlan0 up &
sudo ifconfig bat0 up &
sudo ifconfig bat0 192.168.123.1/24 &



exit 0

```

## Assegnazione di un indirizzo ipv4 statico 
1. Andare nel seguente file: 
***
```sudo nano /etc/dhcpcd.conf```

2. Inserire il comando ```static ip_address=192.168.xxx.4/24 ``` 




# Comunicazione con ZeroMQ

Sfruttando zeroMQ è possibile creare il protocollo di comunicazione e l'app di messagistica che verrà usata per svolgere l'esperimento condividendo le informazioni dello stato dei vari robot a vicenda.


Un socket di tipo PUB-SUB è stato creato: in ogni macchina girano in contemporanea due programmi: un primo funge da server, è un PUB raccoglie le letture dei sensori, le impacchetta e le spedisce ai SUBs, che usano questi valori con l'algoritmo di MDS. 

5 server e 5 subscriber sono quindi necessari. Il subscriber è lo stesso per tutti gli agenti. La discovery (assegnazione egli indirizzi ip e del canale tcp) viene fatta manuamente, sfruttando l'impostazione statica della rete di droni fatta con Batman-adv. 




# Comunicazione seriale
pip install pyserial

lsusb

dmesg | grep "tty"  e trova il nome della porta

## To check the connected devices
sudo arp-scan --interface=bat0 --localnet

## To check active process running on the raspberry
ps aux


Il comando è questo e va in **sudo nano rc.local**
```/usr/bin/python3 /home/pi/Desktop/example.py &amp; ``` 

- Script di Sub sul mio nodo Raspberry per raccogliere tutte le info e fare dei plot oppure per usare il gateway come  ponte per la comunicazione con il pc. 
Ho da qualche parte uno script che fa plot dinamici con flussi di data in ingresso. 

- Bisogna fare una analisi delle incertezze dei segnali e delle misure, in particolare di quello della velocità linerare. 
Gli altri che poi mi servono sono Giroscopio e MDS. 





# Descrizione cartelle
In **serial_data** ci sono dei log in txt dei robottini e poi in plot_log.py questi sono stampati. Si vede bene che funziona correttamente il gyro, che l'accelerometro aquisisce in modo abbastanza rumoro e che la velocità non è calcolata correttamente al momento. 

In **Roba buona** c'è il codice che funziona per far andare la comunicazione. Forse gli esperimenti che avevo fatto a inizio maggio con il doppio pub sub possono essere rivisti. Li c'era del bel codice.


# Encoder
Vedi come funzionano e come fa la stima, con il feedback  nell'encoder, a questo link ci sono le informazion
https://category.yahboom.net/products/encoder-tt-motor


# Central control room
The ojective is to control all the Mbots from a central deliver controller, in order to simplify the start, stop and reset of the Mbots operations.

In **command_central.py** there is a PUB socket, with an INPUT from keyboard the program send at the Pi connected and waiting from instrunctions the action to perform:
Start, Stop or Reset for power up, terminate or reinitilize

In **initial_script.py** there is a script that must be executed when the Pi is launched: necessary to insert the instruction and execute at reboot. 
It attend for the command from the central station and can start the arduino wired connected, stop it and reset. Furthemore it starts the pythons code PUB and SUB to record the data of the experiment. 


C'è uno script di Arduino, da integrare che accetta comandi dal seriale e spero di poter così sia comandare che leggere in dietro i dati che mi da l'arduino per il raspberry. 


## IMPORTANTE CAPIRE COME FAR IMPORTARE CORRETTAMENTE PYZMQ all'avvio!

il  programma di subscrption che gira sugli Mbot ha come unica cosa che lo fa andare il fatto di ricevere una informazione da qualsiasi veicolo. lui la prende e la  smista e se la salva in archivio. devo allocare tutte le matrici per cercare di renderlo più fluido altrimenti con tutti questi append è un macello.

Mentre il publisher può essere pilotato da esterno, perchè ha seconda dello stato dei robot è lui che manda o no la comunicazione. quindi quando è in stop il publisher, è in stop anche il programma che registra e possono essere in stop anche le macchinette. la funzione che publica è esterna in un altro file, mentre al suo interno si connette al central_pub per ascoltare cosa deve fare. 



# Procedura di Uso

## Setup Raspberry e Arduino
1. Caricare in raspberry che si usa come centrale di comando gli script: 
- Command_central_Gateway.py
- global_variable_Gateway.py  
- central_pub_Gateway.py
- var_strings_Gateway_.py

2. Caricare il codice su tutte le Arduino Uno aka Makeblock Me Orion

3. Caricare su tutte le Raspberry montate negli MBots gli script:

Usare scp to copiare intera cartella:

    scp -r [source_directory] [username]@[destination_host]:[destination_directory]

Per la 1:
```
        scp -r /home/piktessa/Distributed/final_dec/mbots_droni_p2p/ pi@192.168.123.77:/home/pi/Distributed
```
Per la 2:
```
        scp -r /home/piktessa/Distributed/final_dec/mbots_droni_p2p/ pi@192.168.123.9:/home/pi/Distributed
```
Per la 3:
```   
   scp -r /home/piktessa/Distributed/final_dec/mbots_droni_p2p/ pi@192.168.123.5:/home/pi/Distributed
```
Per la 4:
```
        scp -r /home/piktessa/Distributed/final_dec/mbots_droni_p2p/ pi@192.168.123.6:/home/pi/Distributed
```    

- initial_script_3_Mbot.py
- pub_to_sub_Mbot.py 
- subscriber_to_mbot_3_Mbot.py.
- var_strings_.py
- uwb_routine.pu

Codice **uwb_routine.py** contiene codice da inserire in ciascuna Mbot, questo è fatto per
leggere in background l'uwb e salvare i valori in un pickle file, che poi verrà letto da subscriber. 
Semplifico e rendo più efficiente così subscriber. In subscriber devi scriver un pickle file stato quando il 
codice termina, questo codice interrompe esecuzione del codice. 



4. Trovare indirizzi ip, commando **sudo arp-scan --interface=bat0 --localnet**. 
Su **tutte** le raspberry assegna correttamente gli indirizzi e le porte in **var_strings_.py** . 

5. Su ogni Mbot lanciare sia initial_script_3_Mbot.py che subscriber_to_mbot_3_Mbot.py


6. lanciare command central e iniviare comando calibrate, poi start

7. Acquisire tutti i dati

8. Copiare il log in Central Raspberry using  **scp data_Mbot_00.txt piktessa@192.168.123.1:/home/piktessa/Distributed/log_Mbot_6**

**Per la 1:**

    scp data_Mbot_100.txt piktessa@192.168.123.1:/home/piktessa/Distributed/num_1
    scp data_Mbot_12_18_2023__19_47_59.txt piktessa@192.168.123.1:/home/piktessa/Distributed/num_1
    scp logs/data_Mbot_rotation30.txt piktessa@192.168.123.1:/home/piktessa/Distributed/num_1
**Per la 2:**

    scp data_Mbot_100.txt piktessa@192.168.123.1:/home/piktessa/Distributed/num_2
    
    scp logs/data_Mbot_rotation00.txt piktessa@192.168.123.1:/home/piktessa/Distributed/num_2

**Per la 3:**
    scp data_Mbot_100.txt piktessa@192.168.123.1:/home/piktessa/Distributed/num_3
    scp logs/data_Mbot_rotation30.txt piktessa@192.168.123.1:/home/piktessa/Distributed/num_3

**Per la 4:**
    scp data_Mbot_100.txt piktessa@192.168.123.1:/home/piktessa/Distributed/num_4
    scp logs/data_Mbot_rotation00.txt piktessa@192.168.123.1:/home/piktessa/Distributed/num_4

    Controllare la 1, non ha trasmesso, ha degli errori sul seriale in lettura della UWB. 

# Per la relazione: 

1. Acquisizione dati e confronto con Luca per post processing offline

2. Caratterizzazione sensori: accelerometro, encoder, UWB. ==> Farlo ruotare di tot angolo e misura. 

3. Scrittura Abstrract, Introduzione, Formulazione del Problema 

4. Presentazione del lavoro fatto per il design della comunicazione, quindi studio delle tecnologie di comunicazione

5. Presentazione teoria di un differential drive robot, dinamica e quali sono i sensori che abbiamo usato per sviluppare una logica di movimento e controllo

6. Come è stato implementata la comunicazione, Batman-adv e ZeroMQ

7. Caratterizzazione dei sensori e delle misure raccolte, il giro e lo slalom, per IMU e rotary encoder. Una misura ferma e una misura in movimento per gli UWB ? ==> Va bene per caratterizzarli? chiedere a Luca. 


# Studio valori encoder ruote

In Data_giroscopio Mbot1 si vede che la ruota 2 o destra ha una misura più corretta, mentre l'encoder di sinistra a questa velocità ha un offset di 0.5 rad/s


# Distributed Extended Kalman Filter

Ogni Mbot esegue un Extend Kalman Filter, con il suo modello e va a fondere i dati presi dalle ancore fisse, con quelli rilevati dagli Mbot in movimento. 

Per fare questo devo prima risolvere con WLS la trilateration usando i dati che ho a disposizione, cioè misuri da punti fissi e misure da punti mobili.

Da lettura UWB otterrò array con tutte le distanze e uso quello per implementare prima trilateration e poi stima della trilateration nel filtro?  Sicuramente funziona. Ho un dubbio sul suo comportamento poi nella stima della Update. 

- Togliere dati che non servono dall'essere inviati. Si devono mandare i dati della UWB e i dati della posizione istantanea in cui si trovano calcolata col il loro kalman filter. 

## Parte Update del Kalman filter

Io voglio stimare con i miei sensori uwb la posizione (x,y) nel piano di una macchina. 
Risolvo con WLS il problema di trilateration e quel dato lo inserisco nel predict del kalman filter. 

# WLS con le Mbot

Codice sviluppato in **subscriber_to_mbot_3WLS.py**. Ci sono 4 macchine in movimento e una fissa che funge da punto noto,
base operativa. Attraverso wls di ogni macchina, si comunicano la distanza reciproca tra di loro con ancore UWB e la stima 
che ogni macchina sta facendo della sua posizione. Algoritmo ricorsivo che ad ogni iterazione che fa tende a migliorare la sua stima. 
Quindi importante fare la parte in cui nei messaggi si mandano la posizione anche, è da aggiungere a tutto il resto
e poi che con ogni iterazione faccia qualche step per l'algoritmo ricorsivo in più



## fare funzione che calcola varianza errore di misura UWB dopo 1000 acquisizioni
Non ha senso fare una funzione che calcoli la posizione all'inizio in modo statica, 
dovrebbe in meno di un secondo riuscire a fare circa una ventina di iterazioni e 
queste dovrebbero essere sufficienti per arrivare alla stima iniziale. 