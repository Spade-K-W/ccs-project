#ifndef TASK_H
#define TASK_H

/* 初始化三按键循迹任务状态机 */
void task_init(void);

/* 主循环每拍调用一次：读按键并执行当前循迹任务 */
void task_step(void);

#endif /* TASK_H */
