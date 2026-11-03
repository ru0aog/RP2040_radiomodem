// src/trx/tx_job.h  
// Заявка на передачу: CLI/CAT ставит, loop() исполняет.  
// Аналог force_*_transmission маяка, но одна заявка на все моды.  
  
#ifndef TX_JOB_H  
#define TX_JOB_H  
  
#include <Arduino.h>  
#include <stdint.h>  
#include "../modes/modes.h"  
  
#define TX_JOB_PAYLOAD_MAX 256  
  
enum TxJobState : uint8_t {  
    TXJ_IDLE = 0,  
    TXJ_QUEUED,  
    TXJ_ACTIVE,  
    TXJ_DONE,  
    TXJ_ABORTED,  
    TXJ_ERROR  
};  
  
struct TxJob {  
    volatile TxJobState state;  
    ModeId   mode;  
    uint32_t freq_hz;  
    char     payload[TX_JOB_PAYLOAD_MAX];  
    uint32_t started_ms;  
    int      error;          // 0=ok, <0 код ошибки  
};  
  
// Постановка заявки. Возвращает false, если занято/мода без tx_frame.  
bool tx_job_submit(ModeId mode, uint32_t freq_hz, const char* payload);  
  
// Опрос из loop() (Core 0): запускает и завершает сеанс передачи.  
void tx_job_poll(void);  
  
// Запрос на прерывание (из CLI "tx off" или при записи с ПК).  
void tx_job_abort(void);  
  
TxJobState tx_job_state(void);  
const TxJob* tx_job_info(void);  
  
#endif // TX_JOB_H