import ply.lex as lex
import ply.yacc as yacc
from pprint import pprint

# List of token names.   This is always required
tokens = ("OBRACE",
          "CBRACE",
          "SEMI",
          "COMMA",
          "OPAR",
          "CPAR",
          "QUOTE",
          "IMPORT",
          "COMMUNICATIONS",
          "LANGUAGE",
          "COMPONENT",
          "PYTHON",
          "CPP",
          'CPP11',
          "GUI",
          "QWIDGET",
          "QMAINWINDOW",
          "QDIALOG",
          "QT",
          "REQUIRES",
          "IMPLEMENTS",
          "SUBSCRIBESTO",
          "PUBLISHES",
          "OPTIONS",
          "INNERMODELVIEWER",
          "STATEMACHINE",
          "VISUAL",
          "AGMAGENT",
          "AGM2AGENT",
          "AGM2AGENTROS",
          "AGM2AGENTICE",
          "ICE",
          "ROS",
          "IDENTIFIER",
          "PATH")


# Regular expression rules for simple tokens
t_OBRACE = r'\{'
t_CBRACE = r'\}'
t_SEMI = r';'
t_COMMA= r','
t_OPAR = r'\('
t_CPAR = r'\)'
t_QUOTE = r'\"'
t_IDENTIFIER = r'(?i)[A-Za-z0-9\_]+'


# Moved here because a priority problem making this to be interpreted as t_IDENTIFIER
# https://stackoverflow.com/questions/2910338/python-yacc-lexer-token-priority
def t_PATH(t):
    r'(?i)[a-zA-Z0-9]+\.[a-zA-Z0-9]+'
    return t

def t_IMPORT(t):
    r'(?i)IMPORT'
    return t

def t_PYTHON(t):
    r'(?i)PYTHON'
    return t

def t_CPP11(t):
    r'(?i)CPP11'
    return t

def t_CPP(t):
    r'(?i)CPP'
    return t

def t_GUI(t):
    r'(?i)GUI'
    return t

def t_QWIDGET(t):
    r'(?i)QWIDGET'
    return t

def t_QMAINWINDOW(t):
    r'(?i)QMAINWINDOW'
    return t

def t_QDIALOG(t):
    r'(?i)QDIALOG'
    return t

def t_QT(t):
    r'(?i)QT'
    return t

def t_REQUIRES(t):
    r'(?i)REQUIRES'
    return t

def t_IMPLEMENTS(t):
    r'(?i)IMPLEMENTS'
    return t

def t_SUBSCRIBESTO(t):
    r'(?i)SUBSCRIBESTO'
    return t

def t_PUBLISHES(t):
    r'(?i)PUBLISHES'
    return t

def t_OPTIONS(t):
    r'(?i)OPTIONS'
    return t

def t_INNERMODELVIEWER(t):
    r'(?i)INNERMODELVIEWER'
    return t

def t_STATEMACHINE(t):
    r'(?i)STATEMACHINE'
    return t

def t_VISUAL(t):
    r'(?i)VISUAL'
    return t

def t_AGM2AGENTICE(t):
    r'(?i)AGM2AGENTICE'
    return t

def t_AGM2AGENTROS(t):
    r'(?i)AGM2AGENTROS'
    return t

def t_AGM2AGENT(t):
    r'(?i)AGM2AGENT'
    return t

def t_AGMAGENT(t):
    r'(?i)AGMAGENT'
    return t

def t_ICE(t):
    r'(?i)ICE'
    return t

def t_ROS(t):
    r'(?i)ROS'
    return t

def t_LANGUAGE(t):
    r'(?i)LANGUAGE'
    return t

def t_COMMUNICATIONS(t):
    r'(?i)COMMUNICATIONS'
    return t

def t_COMPONENT(t):
    r'(?i)COMPONENT'
    return t


# Define a rule so we can track line numbers
def t_newline(t):
    r'\n+'
    t.lexer.lineno += len(t.value)


# A string containing ignored characters (spaces and tabs)
t_ignore = ' \t'


# Error handling rule
def t_error(t):
    print("Illegal character '%s'" % t.value[0])
    t.lexer.skip(1)


# Build the lexer
lexer = lex.lex()

def p_component(p):
    '''component : importslist COMPONENT IDENTIFIER OBRACE content CBRACE SEMI'''
    p[0] = {"name": p[3], "imports": p[1]}
    p[0].update(p[5])

def p_importslist(p):
    '''
    importslist  : idslimport importslist
                 |
    '''
    if len(p) > 2:
        p[0] = p[2]
        p[0].append(p[1])
    elif len(p) > 1:
        p[0] = [p[1]]
    else:
        p[0] = []

def p_idslimport(p) :
    '''
    idslimport  : IMPORT QUOTE PATH QUOTE SEMI
    '''
    if len(p)>1:
        p[0] = p[3]
    else:
        p[0] = []

def p_content(p):
    '''content  : commblock language otherslist'''
    p[0] = {'language': p[2]}
    p[0].update(p[1])
    if p[3] is not None:
        p[0].update(p[3])

def p_language(p):
    '''language	: LANGUAGE langopts SEMI'''
    p[0] = p[2]

def p_langopts(p):
    '''
    langopts    : CPP
                | CPP11
                | PYTHON
    '''
    p[0] = p[1]

def p_otherslist(p):
    '''
    otherslist  : otherslist others
                |
    '''
    # if there's others statements repeated, the last one will be taken
    if len(p) > 2:
        p[0] = p[1]
        p[0].update(p[2])
    else:
        p[0] = {}


def p_others(p):
    '''
    others  : gui
            | statemachine
            | options
    '''
    p[0] = p[1]

def p_gui(p):
    '''
    gui		: GUI QT OPAR guiopts CPAR SEMI
    '''
    if len(p)> 1:
        p[0] = {'gui': [p[1], p[4]]}
    else:
        p[0] = {'gui': None}

def p_guiopts(p):
    '''
    guiopts	: QWIDGET
            | QMAINWINDOW
            | QDIALOG
    '''
    p[0] = p[1]

def p_options(p):
    '''
    options : OPTIONS optionsoptlist SEMI
    '''
    if len(p)>1:
        p[0] = p[2]
    else:
        p[0] = []

def p_optionsoptlist(p):
    '''
    optionsoptlist  : optionsoptlist COMMA optionsopt
                    | optionsopt
    '''
    if len(p) > 2:
        p[0] = p[1]
        p[0]['options'].append(p[3])
    elif len(p) > 1:
        p[0] = {'options': [p[1]]}
    else:
        p[0] = {'options': []}

def p_optionsopt(p):
    '''
    optionsopt	: INNERMODELVIEWER
                | AGMAGENT
                | AGM2AGENT
                | AGM2AGENTICE
                | AGM2AGENTROS
    '''
    p[0] = p[1]

def p_statemachine(p):
    '''
    statemachine    : STATEMACHINE QUOTE PATH QUOTE stmvisual SEMI
    '''
    if len(p) > 1:
        p[0] = {'statemachine': {'path': p[3], 'visual': p[5]}}
    else:
        p[0] = {'statemachine': None}

def p_stmvisual(p):
    '''
    stmvisual   : VISUAL
                |
    '''
    if len(p) == 2:
        p[0] = True
    else:
        p[0] = False


def p_commblock(p):
    '''commblock	: COMMUNICATIONS OBRACE commlist CBRACE SEMI'''
    p[0] = p[3]

def p_commlist(p):
    '''commlist : commlist comm
                | '''
    if len(p) > 2:
        p[0] = p[1]
        # in case of repetitions of the same commkeywords
        if all(key in p[0] for key in p[2].keys()):
            for key in p[2].keys():
                p[0][key].extend(p[2][key])
        else:
            p[0].update(p[2])
    else:
        p[0] = {}


def p_comm(p):
    '''comm : commkeywords commalist SEMI'''
    if len(p)> 2:
        p[0] = {p[1].lower():p[2]}

def p_commkeywords(p):
    '''commkeywords : IMPLEMENTS
                    | REQUIRES
		            | SUBSCRIBESTO
		            | PUBLISHES'''
    p[0] = p[1]

def p_commalist(p):
    '''commalist    : commalist COMMA commitem
                    | commitem'''
    if len(p) > 2:
        p[0] = p[1]
        p[0].append(p[3])
    else:
        p[0] = [p[1]]


def p_commitem(p):
    '''
    commitem    : IDENTIFIER commtype
                | IDENTIFIER
    '''
    p[0] = {'name': p[1]}
    if len(p) > 2:
        p[0]['type'] = p[2]
    else:
        p[0]['type'] = 'ice'


def p_commtype(p):
    '''commtype : OPAR ICE CPAR
                | OPAR ROS CPAR
    '''
    p[0] = p[2]


def p_error(p):
  print("Syntax error at %s"%p.value)

# def p_empty(p):
#     '''empty : '''
#     pass

# Build the parser
bparser = yacc.yacc()

###  TESTING CODE
bparser.error = 0
data = '''
import "blabblaba1.idsl";
import "blabblaba2.idsl";
import "blabblaba3.idsl";
component lamacusa
{
    communications
    {
        implements nube(ros), manzana(ice), pera;
        requires nube2(ros), manzana2(ice), pera2;
    };
    language cpp11;
    options innermodelviewer;
    gui qt (qwidget);
    statemachine "casalama.smdsl" visual;
};
'''

p = bparser.parse(data)
pprint(p)