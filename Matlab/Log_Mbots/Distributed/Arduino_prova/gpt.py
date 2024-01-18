import sys
import select

def input_with_timeout(timeout):
    #print(prompt, end='', flush=True)
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    if ready:
        return sys.stdin.readline().rstrip()
    else:
        return None

while True:
    comando = input("Inserisci un comando (o 'fine' per terminare): ")

    if comando == 'fine':
        break  # Esci dal ciclo while esterno

    if comando == 'inizio':
        a = 0 
        while True:
            # Esegui le azioni nel loop interno
            # ...
            #print(a)
            nuovo_input = input_with_timeout(timeout=1)
            
            
            if nuovo_input is None:
                print("none ", a)
                # Nessun input ricevuto entro il timeout
                # Continua l'esecuzione del loop interno
                pass
            elif nuovo_input == 'break':
                break  # Esci dal ciclo while interno
            else:
                # Esegui le azioni corrispondenti al nuovo input
                # ...

                # Stampa un messaggio o esegui altre azioni
                print("Nuovo input eseguito:", nuovo_input)
                a = nuovo_input
                print("nuovo ", a)