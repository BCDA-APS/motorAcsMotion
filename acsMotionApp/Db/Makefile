TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

#----------------------------------------------------
#  Optimization of db files using dbst (DEFAULT: NO)
#DB_OPT = YES

#----------------------------------------------------
# Create and install (or just install) into <top>/db
# databases, templates, substitutions like this
#DB += xxx.db

DB += SPiiPlusAuxAi.db
DB += SPiiPlusAuxAo.db
DB += SPiiPlusAuxBi.db
DB += SPiiPlusAuxBo.db
DB += SPiiPlusAuxLi.db
DB += SPiiPlusAuxLo.db

DB += SPiiPlusJogging.db
DB += SPiiPlusHoming.db
DB += SPiiPlusCommut.db
DB += SPiiPlusMaxParams.db
DB += SPiiPlusMaxParamsRbv.db
DB += SPiiPlusIntVar.db
DB += SPiiPlusRealVar.db
DB += SPiiPlusProgram.db
DB += SPiiPlusAxisExtra.db
DB += SPiiPlusFeedback.db
DB += SPiiPlusDisableSetPos.db
DB += SPiiPlusTest.db

#----------------------------------------------------
# If <anyname>.db template is not named <anyname>*.template add
# <anyname>_template = <templatename>

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

