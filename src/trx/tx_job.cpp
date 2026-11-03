// src/trx/tx_job.cpp  
// Сеанс передачи по заявке: trx_switch(TX) -> tx_frame -> trx_switch(RX).  
// Порядок как в маяке: подготовка -> передача -> гарантированный возврат в RX.  
  
#include "tx_job.h"  
#include "trx_control.h"  
#include "../filesys/file_manager.h"   // my_freq_rx_var, pc_file_written  
#include "../vfo/vfo.h"                // vfo_boost_enter/leave если есть  
  
static TxJob s_job;  
  
bool tx_job_submit(ModeId mode, uint32_t freq_hz, const char* payload) {  
    if (s_job.state == TXJ_QUEUED || s_job.state == TXJ_ACTIVE) return false;  
    if (trx_state() != TRX_RX) return false;  
  
    const ModeIface* mi = modes_get(mode);  
    if (!mi || !mi->tx_frame) return false;  
  
    s_job.mode    = mode;  
    s_job.freq_hz = freq_hz;  
    s_job.error   = 0;  
    strncpy(s_job.payload, payload ? payload : "", TX_JOB_PAYLOAD_MAX - 1);  
    s_job.payload[TX_JOB_PAYLOAD_MAX - 1] = '\0';  
    s_job.state = TXJ_QUEUED;  
    return true;  
}  
  
void tx_job_abort(void) {  
    if (s_job.state == TXJ_QUEUED || s_job.state == TXJ_ACTIVE)  
        s_job.state = TXJ_ABORTED;  
}  
  
TxJobState tx_job_state(void) { return s_job.state; }  
const TxJob* tx_job_info(void) { return &s_job; }  
  
void tx_job_poll(void) {  
    switch (s_job.state) {  
  
    case TXJ_QUEUED: {  
        // Запрет записи flash на весь сеанс — гейтинг trx_state()==TRX_RX  
        // в file_manager сработает после trx_switch.  
        if (!trx_switch(TRX_TX, s_job.freq_hz)) {  
            s_job.error = -1;  
            s_job.state = TXJ_ERROR;  
            Serial.println("[TX] trx_switch(TX) failed");  
            break;  
        }  
        s_job.started_ms = millis();  
        s_job.state = TXJ_ACTIVE;  
        Serial.printf("[TX] start mode=%d f=%lu\n",  
                      (int)s_job.mode, (unsigned long)s_job.freq_hz);  
        break;  
    }  
  
    case TXJ_ACTIVE: {  
        // Прерывание по записи с ПК — как в маяке (pc_file_written).  
        if (pc_file_written) s_job.state = TXJ_ABORTED;  
  
        const ModeIface* mi = modes_get(s_job.mode);  
        if (s_job.state == TXJ_ABORTED) {  
            Serial.println("[TX] aborted");  
        } else if (mi && mi->tx_frame) {  
            // Блокирующая передача кадра (как send_*_string в маяке).  
            // Внутри tx_frame желательна проверка tx_job_state()==TXJ_ABORTED.  
            mi->tx_frame(s_job.payload, strlen(s_job.payload));  
        }  
  
        // Гарантированный возврат в RX — как в маяке.  
        trx_switch(TRX_RX, (uint32_t)my_freq_rx_var.toInt());  
        s_job.state = (s_job.state == TXJ_ABORTED) ? TXJ_ABORTED : TXJ_DONE;  
        Serial.printf("[TX] done in %lu ms\n",  
                      (unsigned long)(millis() - s_job.started_ms));  
        break;  
    }  
  
    case TXJ_DONE:  
    case TXJ_ABORTED:  
    case TXJ_ERROR:  
        s_job.state = TXJ_IDLE;   // однократный показ результата  
        break;  
  
    default:  
        break;  
    }  
}