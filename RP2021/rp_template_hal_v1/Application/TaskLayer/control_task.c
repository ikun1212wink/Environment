/**
 *  @file       control_task.c
 *  @author     @RobotPilots
 *  @version    v1.0
 *  @brief      Control Center
 *  @update     
 *              v1.0(9-November-2020)
 */

/* Includes ------------------------------------------------------------------*/
#include "control_task.h"

#include "chassis.h"
#include "cmsis_os.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/**
 *  @brief  ¿ØÖÆÈÎÎñ
 */

void StartControlTask(void const * argument)
{
    chassis.init();
    for(;;)
    {
        if(sys.state == SYS_STATE_NORMAL) {
            chassis.ctrl();
        } else {
            chassis.self_protect();
        }
        
        osDelay(2);
    }
}
