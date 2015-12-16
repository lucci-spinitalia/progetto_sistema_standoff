include "segway_config.h" 
#include "segway_udp_v2.h" 

#define JOY_MAX_VALUE 32767 

#define SEGWAY_ADDRESS "192.168.1.40" //indirizzo del veicolo  
#define SEGWAY_PORT 55                // porta udp del veicolo
#define SCU_SEGWAY_PORT 8017  // where to send segway info
#define CCU_SEGWAY_PORT 8003  // where to send segway info
#define SEGWAY_INACTIVITY_TIMEOUT_SEC 2 // tempo di inattività dopo il quale il veicolo viene considerato offline

/* State request */
#define SCU_ADDRESS "192.168.1.28"
#define SCU_STATE_PORT 8016
#define CCU_STATE_PORT 8004  // where to send state info

/* Timeout */
#define TIMEOUT_SEC 0 
#define SEGWAY_TIMEOUT_USEC 100000  // tempo di refresh del comando verso il veicolo in microsecondi
#define SCU_STATE_TIMEOUT_USEC 500000 // tempo di aggiornamento dei messaggi per la SCU

#define SCU_STATE_TIMEOUT_LIMIT_SEC 5 // tempo di inattività oltre il quale la SCU viene considerata offline

int main()
{
  union segway_union segway_status;  //contiene tutti i parametri riguardanti il veicolo
  
  /* INIZIALIZZAZIONE */
  Verifica che la connessione sia disponibile altrimenti
    esci dal programma
    
  segway_socket = Apri il socket verso il veicolo
  
  Se l'operazione non è andata a buon fine
  {
    Gestisci errore
    segway_status.list.operational_state = UNKNOWN;
  }
  
  Inizializza il joystick
  
  Apri il socket per lo scambio dello stato/richieste con la SCU (SCU_ADDRESS:SCU_STATE_PORT) e gestisci eventuali errori
  
  Apri il socket per lo scambio dello stato del veicolo con la SCU (SCU_ADDRESS:SCU_SEGWAY_PORT)
  
  Inizializza i timer
  
  while(1)
  {
    /* GESTIONE MESSAGGI DALLA SCU PER QUANTO RIGUARDA LO STATO */
    Se è stato ricevuto un messaggio dalla SCU
    {
      Azzera contatore assenza segnale SCU
      Aggiorna lo stato della SCU sul display
    }
  
  
    /* GESTIONE MESSAGGI DA E VERSO IL VEICOLO */
    // Lettura dei messaggi dal veicolo
    bytes_read = segway_read(segway_socket, &segway_status, NULL);
    
    Se bytes_read <= 0
      Gestisci errore
    altrimenti
    {
      Azzera contatore di assenza segnale veicolo
      Aggiorna stato segway sul display a seconda dello stato segway_status.list.operational_state
      // converte la struttura ccu_segway_info, contenente lo stato del segway, in un array
      // di byte, pronti per essere spediti via udp
      segway_convert_param_message(ccu_segway_info, ccu_buffer, &ccu_buffer_size);
      
      Rinvia lo stato del segway alla CCU tramite la CCU_SEGWAY_PORT
      Rinvia lo stato del segway alla SCU SCU_ADDRESS:SCU_SEGWAY_PORT
    }
  
    // Invio dati verso il veicolo
    Se sono trascorsi SEGWAY_TIMEOUT_USEC
    {
      segway_send(segway_socket, &segway_address)
    }
    
    
    
    
    /* GESTIONE MESSAGGI DAL JOYSTICK */
    Se lo stato del joystick è cambiato
    {
      Aggiorna i valori riferiti al joystick
      Aggiorna flag di richiesta
     } 
      
      
      
    /* ZONA TIMER */
    Se timer segway scaduto (SEGWAY_TIMEOUT_USEC)
    {
      Incrementa contatore inattività veicolo
      
      Se il contatore è maggiore di SEGWAY_INACTIVITY_TIMEOUT_SEC
      {
        Segnala che il veicolo è offline
        
        Invia il nuovo stato alla CCU tramite la CCU_SEGWAY_PORT
        
        Invia il nuovo stato alla SCU SCU_ADDRESS:SCU_SEGWAY_PORT
      }
      
      // Gestisce i messaggi da inviare al veicolo a seconda dello stato del sistema
      segway_status_update(segway_socket, &segway_address, &segway_status,  scu_state, &jse, JOY_MAX_VALUE);
    }
    
    
    
    Se timer scu_state scaduto (SCU_STATE_TIMEOUT_USEC)
    {
      Invia un messaggio alla SCU_ADDRESS:SCU_STATE_PORT
      
      Incrementa contatore inattività scu
      
      Se contattore > SCU_STATE_TIMEOUT_LIMIT_SEC
      {
        Aggiorna lo stato della scu a SCU_ARM_UNKNOWN
        
        Aggiorna lo stato sul display
        
        Invia lo stato della scu alla ccu tramite CCU_STATE_PORT
      }
    }
  }
}




segway_status_update()
{
  switch(segway_status->list.operational_state)
}