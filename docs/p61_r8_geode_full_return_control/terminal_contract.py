"""Classify recorded production outcomes, never manufacture an ACK/pose."""
TIME_FAILURES={'NO_HISTORY','FUTURE_MEASUREMENT','INVALID_MEASUREMENT',
               'REPLAY_INCOMPLETE','ALIGNMENT_TOO_LARGE','INVALID_TIMESTAMP'}


def terminal(*, state_valid, prediction_valid, deskew, ndt, oosm=None,
             fusion_applied=None, fresh_corrected=False, times_valid=True):
    if not state_valid:return 'NUMERICAL_FAILURE',True
    if not times_valid:return 'TIME_CONTRACT_BLOCKED',True
    if deskew=='REJECTED':return 'DESKEW_REJECTED',True
    if deskew!='PUBLISHED' or ndt is None:return None,False
    # Recorder TCPROS delivery can lag the production callback: wait to deadline.
    if not prediction_valid:return None,False
    if not ndt:return 'NDT_FAILED_PREDICTION_ONLY',False
    if oosm in TIME_FAILURES:return oosm,True
    if oosm is None or fusion_applied is None:return None,False
    if oosm=='APPLIED':
        if not fusion_applied:return 'FUSION_RECEIPT_CONFLICT',True
        return ('APPLIED',False) if fresh_corrected else (None,False)
    if fusion_applied:return 'FUSION_RECEIPT_CONFLICT',True
    return 'FUSION_REJECTED_'+oosm,False
