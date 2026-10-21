# 预测合同：NOT_EVALUATED_AFTER_ASSET_BLOCK

没有读取wheel消息或建立测试预测。不存在以测试GT替代预测的结果。

批准但未执行的计划是仅在最后合法建图时刻t0作一次GT锚定：

`T_W_B_pred(t)=T_W_B_GT(t0) inverse(T_O_B(t0)) T_O_B(t)`；
`T_M_L_pred(t)=T_W_B_pred(t) T_B_L`。

必须先认证wheel实际child/body语义及外参方向才能应用公式。当前未认证，也未假定wheel frame等于动捕刚体。未用测试GT、NDT结果或未来状态构造预测。
