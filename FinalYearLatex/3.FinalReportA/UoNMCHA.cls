%%
%% LaTeX document style `UoNMCHA', for use with LaTeX2e

\NeedsTeXFormat{LaTeX2e}
\LoadClass[twoside,a4paper,11pt]{scrartcl}

\RequirePackage[a4paper,top=2cm,left=2cm,right=2cm,bottom=2cm,includefoot,includehead]{geometry} 
\RequirePackage{fancyhdr}

\RequirePackage{abstract}
%\RequirePackage{natbib}
\RequirePackage{lastpage}
\RequirePackage{url}
\RequirePackage{graphicx} % For graphics

\AtBeginDocument{\setcounter{page}{\@firstpage}}

\newsavebox{\MIClogo}
\sbox{\MIClogo}{\includegraphics[width=3cm]{UoN_Logo}}

\pagestyle{fancy}
%% Ordinary pages
\fancyhf{}
\fancyhead[CE]{School of Engineering, Discipline of Mechanical Engineering and Mechatronics}
\fancyhead[CO]{\usebox{\shorttitlebox}}
\fancyfoot[LE]{\thepage}
\fancyfoot[RO]{\thepage}
%% First page
\fancypagestyle{plain}{
\fancyhf{}
\fancyhead[L]{School of Engineering, Discipline of Mechanical Engineering and Mechatronics,  pp. \@firstpage--\pageref{LastPage}}
\fancyhead[R]{\setlength{\unitlength}{1cm}
  \begin{picture}(0,0)
    \put(-1.8,-1.5){\usebox{\MIClogo}}
  \end{picture}
}
\fancyfoot[L]{\footnotesize The University of Newcastle, AUSTRALIA}
\fancyfoot[R]{\footnotesize  \@UoNMCHAreportNo}
%\fancyfoot[C]{\thepage}
\renewcommand{\headrulewidth}{0pt}}

\def\@titlesize{\huge}
\def\@undertitleskip{\z@}

\newcommand{\firstpage}[1]{\newcommand{\@firstpage}{#1}}
%\newcommand{\MICvolume}[1]{\newcommand{\@MICvolume}{#1}}
\newcommand{\UoNMCHAreportNo}[1]{\newcommand{\@UoNMCHAreportNo}{#1}}

%\newcommand{\MICissue}[1]{\newcommand{\@MICissue}{#1}}
\newcommand{\UoNMCHAyear}[1]{\newcommand{\@UoNMCHAyear}{#1}}
\newcommand{\shorttitle}[1]{\savebox{\shorttitlebox}{#1}}

% The following is mostly stolen from scrartcl.cls, in order to
% redefine the layout of the authors. Use of saveboxes to store layout
% of authors and addresses added.

\renewcommand*\maketitle[1][1]{\par
  \@tempcnta=#1\relax\ifnum\@tempcnta=1\else
    \ClassWarning{\KOMAClassName}{
      Optional argument of \string\maketitle\space ignored
      at\MessageBreak
      notitlepage-mode
    }
  \fi
  \begingroup
    \renewcommand*\thefootnote{\@fnsymbol\c@footnote}
    \let\@oldmakefnmark\@makefnmark
    \renewcommand*{\@makefnmark}{\rlap\@oldmakefnmark}
% Workaround for bug in KOMA-script?
%%     \if@twocolumn
%%       \ifnum \col@number=\@ne
%%         \@maketitle
%%       \else
        \twocolumn[\@maketitle]
%%       \fi
%%     \else
%%       \newpage
%%       \global\@topnum\z@
%%       \@maketitle
%%     \fi
    \thispagestyle{\titlepagestyle}\@thanks
  \endgroup
  \@madetitletrue
  \setcounter{footnote}{0}
  \let\thanks\relax
  \let\maketitle\relax
  \let\@maketitle\relax
  \global\let\@thanks\@empty
  \global\let\@author\@empty
  \global\let\@date\@empty
  \global\let\@title\@empty
  \global\let\@extratitle\@empty
  \global\let\@titlehead\@empty
  \global\let\@subject\@empty
  \global\let\@publishers\@empty
  \global\let\@uppertitleback\@empty
  \global\let\@lowertitleback\@empty
  \global\let\@dedication\@empty
  \global\let\author\relax
  \global\let\title\relax
  \global\let\extratitle\relax
  \global\let\titlehead\relax
  \global\let\subject\relax
  \global\let\publishers\relax
  \global\let\uppertitleback\relax
  \global\let\lowertitleback\relax
  \global\let\dedication\relax
  \global\let\date\relax
  \global\let\and\relax
}

\renewcommand*{\@maketitle}{
  \clearpage
  \let\footnote\thanks
  \null
  \vskip 23mm
  \begin{center}
    {\titlefont\huge \@title \par}
    \vskip 1.5em
    {\Large
      \lineskip .5em
      \centerline{\usebox{\authorbox}}
    }
    \vskip 1em
    \list{}{\listparindent20\p@
    \itemindent\z@ \leftmargin.025\linewidth
    \rightmargin\leftmargin \parsep \z@}\item[]\ignorespaces
    {\usebox{\addressbox} \par}%
    \endlist\global\@ignoretrue
   \vskip \z@ \@plus 1em
  \end{center}%
  \par
  \vskip 1em
%%   \twocolumn[%
%%     \centering%
%%     \rule{.95\linewidth}{.8pt}   % full width line 
%%     \begin{onecolabstract}    % full width abstract
%%       \the\abstract@toks
%%     \end{onecolabstract}
%%     \rule{.95\linewidth}{.8pt}%
%%     \vspace{20pt}
%%   ]
%%   \saythanks 
%%  \centerline{\rule{.95\linewidth}{.8pt}}\par
%%  \vskip .5em
%%  \centerline{\bfseries Executive Summary}%
%%  \vskip.5\baselineskip\rmfamily
%%  \list{}{\listparindent20\p@
 %%   \itemindent\z@ \leftmargin.025\linewidth
 %%   \rightmargin\leftmargin \parsep \z@}\item[]\ignorespaces
%%  \the\abstract@toks
%%  \par
%%  \vskip 1em
  %\noindent\emph{Keywords:} \the\keywords@toks
  \endlist\global\@ignoretrue
  \noindent\rule{\linewidth}{.8pt}
  \par
  \vskip 2em
}

% The below is shamelessly stolen from elsart.cls to allow
% specification of \address (must also redefine \author)

\newsavebox{\authorbox}     % For saving authors, to be printed by
                            % \maketitle
\savebox{\authorbox}{}
\newsavebox{\addressbox}    % For saving addresses, to be printed by
                            % \maketitle 
\newsavebox{\abstractbox}   % For saving abstract
\newsavebox{\shorttitlebox} % For saving short title



\def\@authorsize{\Large}
\def\author@font{}
\def\@addressstyle{\small\itshape}

\newif\if@firstauthor             % First author or collaboration?
\newif\if@newelem                 % Switch to new type of element?
\newif\if@madetitle  \@madetitlefalse % Is madetitle run?

\def\author{\@ifnextchar[{\author@optarg}{\author@optarg[]}}
\def\author@optarg[#1]#2{%
  \sbox{\authorbox}{%
    \mbox{%
      \usebox{\authorbox}
      \stepcounter{author}%
      \beg@elem
      \add@tok\email@text{\ead@newau}%
      \add@tok\url@text{\ead@newau}%
      \@for\@tempa:=#1\do{\expandafter\add@thanksref\expandafter{\@tempa}}%
      \report@elt{author}\proc@elem{author}{#2}%
      \ifnum0\n@collab@=\z@ \runningauthor@fmt \fi
      \author@fmt{\the\c@author}{\the\t@loc@notes}{\@author}%
    }
  }
}


\let\report@elt\@gobble
\def\add@tok#1#2{\global#1\expandafter{\the#1#2}}
\def\add@xtok#1#2{\begingroup
  \no@harm
  \xdef\@act{\global\noexpand#1{\the#1#2}}\@act
\endgroup}
\def\beg@elem{\global\t@loc@notes={}\global\note@cnt\z@}
\def\@xnamedef#1{\expandafter\xdef\csname #1\endcsname}
\def\no@harm{%
  \let\\=\relax  \let\rm\relax
  \let\ss=\relax \let\ae=\relax \let\oe=\relax
  \let\AE=\relax \let\OE=\relax
  \let\o=\relax  \let\O=\relax
  \let\i=\relax  \let\j=\relax
  \let\aa=\relax \let\AA=\relax
  \let\l=\relax  \let\L=\relax
  \let\d=\relax  \let\b=\relax \let\c=\relax
  \let\bar=\relax
  \def\protect{\noexpand\protect\noexpand}}
\def\proc@elem#1#2{\begingroup
    \no@harm                             % make a few instructions harmless
    \let\thanksref\@gobble               % remove \thanksref from element
    \let\corauthref\@gobble
    \@xnamedef{@#1}{#2}%                 % and store as \@#1
    \let\thanksref\add@thanksref
    \let\corauthref\add@thanksref
    \setbox\@tempboxa\hbox{#2}%
  \endgroup
  \prev@elem=\cur@elem                   % keep track of type of previous
  \cur@elem=\csname e@#1\endcsname       % and current element
}
\def\add@thanksref#1{\global\advance\note@cnt\@ne
  \ifnum\note@cnt>\@ne \add@xtok\t@loc@notes{\note@sep}\fi
  \add@tok\t@loc@notes{\ref{#1}}}

\let\@ead@au\@empty
\def\ead@newau{\ifx\@ead@au\@empty\else\ead@addau\@ead@au
  \let\@ead@au\@empty\fi}
\let\ead@endau\ead@newau
\def\ead@addau#1{ (#1)}
\def\ead@au#1{\edef\@ead@au{#1}}

\def\runningauthor@fmt{%
  \begingroup\no@harm
    \if@firstauthor
      \ifnum0\n@author@ > 2
        \global\edef\@runningauthor{\@author\ et al.}%
      \else
        \global\let\@runningauthor\@author%
      \fi
    \else % \c@author > 1
      \ifnum0\n@author@ = 2
        \global\edef\@runningauthor{\@runningauthor\ \& \noexpand\@author}%
      \fi
    \fi
  \endgroup
}
\def\author@fmt#1#2#3{\@newelemtrue
  \if@firstauthor
  \first@author \global\@firstauthorfalse \fi
  \ifnum\prev@elem=\e@author \global\@newelemfalse \fi
%  \if@newelem \author@fmt@init \fi
  \@authorsize % Added by Lars
  \edef\@tempb{#2}\ifx\@tempb\@empty
    \hbox{{\author@font #3}}\else
    \hbox{{\author@font #3}\,$^{\mathrm{#2}}$}%
  \fi}
\def\first@author{\author@note@fmt
  \corauth@mark@fmt
  \add@tok\t@glob@notes
  {\output@corauth@text
    \output@ead@text{email}%
    \output@ead@text{url}%
    \author@note@fmt}%
}%
%% \def\author@fmt@init{%
%%   \par
%%   \vskip 8\p@ \@plus 4\p@ \@minus 2\p@
%%   \@authorsize 
%%   \leavevmode}                        % Vertical space above author list
 \def\and{\unskip~and~}
\def\collab{\@ifstar{\collab@arg}{\collab@arg}}
\let\collaboration=\collab
\def\collab@arg#1{\stepcounter{collab}%
  \if@firstauthor \first@collab \global\@firstauthorfalse \fi
  \gdef\@runningauthor{#1}%
  \beg@elem
  \proc@elem{collab}{#1}%
  \collab@fmt{\the\c@collab}{\the\t@loc@notes}{\@collab}%
  \ignorespaces}
\def\collab@fmt#1#2#3{\@newelemtrue
  \ifnum\prev@elem=\e@collab \global\@newelemfalse \fi
  \if@newelem \collab@fmt@init \fi
  \par                                 % Start new paragraph
  {\large #3\,$^{\mathrm{#2}}$}}
\def\first@collab{
  \collab@note@fmt                     % re-define \thefootnote as
  \add@tok\t@glob@notes                % appropriate for collab/address
    {\collab@note@fmt}}%
\def\collab@fmt@init{\vskip 1em}       % Vertical space above list
\def\author@note@fmt{\setcounter{footnote}{0}%
  \def\thefootnote{\xarabic{footnote}}}
\let\collab@note@fmt=\author@note@fmt
\def\corauth@mark@fmt{\def\thecorauth{\astsymbol{corauth}}}
\def\output@corauth@text{\def\thefootnote{\astsymbol{footnote}}%
  \the\corauth@text}

\let\real@refstepcounter\refstepcounter
\def\address{\@ifstar{\address@star}%
    {\@ifnextchar[{\address@optarg}{\address@noptarg}}}
\def\address@optarg[#1]#2{%
  \savebox{\addressbox}{%
    \parbox[t]{.95\textwidth}{%
      \usebox{\addressbox}
      \real@refstepcounter{address}%
      \beg@elem
      \report@elt{address}\proc@elem{address}{#2}%
      \address@fmt{\c@address}{\the\t@loc@notes}{\@address}{#1}%
      \label{#1}
      \ignorespaces%
    }
  }
}
\def\address@noptarg#1{
  \savebox{\addressbox}{%
    \parbox[t]{\textwidth}{%
      \usebox{\addressbox}
      \real@refstepcounter{address}%
      \beg@elem
      \proc@elem{address}{#1}%
      \address@fmt{\z@}{\the\t@loc@notes}{\@address}{\theaddress}%
      \ignorespaces%
    }
  }
}
\def\address@star#1{%
  \beg@elem
  \proc@elem{address}{#1}%
  \address@fmt{\m@ne}{\the\t@loc@notes}{\@address}{*}%
  \ignorespaces}
\def\theaddress{\alph{address}}
\def\address@fmt#1#2#3#4{\@newelemtrue
  \def\@eltag{\theaddress}
  \ifnum\prev@elem=\e@address \@newelemfalse \fi
  \if@newelem \address@fmt@init \fi
  \noindent \bgroup \@addressstyle
  \ifnum#1=\z@
    #3\,$^{\mathrm{#2}}$\space%
  \else
    \ifnum#1=\m@ne
      $^{\phantom{\mathrm{\@eltag}}}$\space #3\,$^{\mathrm{#2}}$%
    \else
      $^{\mathrm{\@eltag}}\space$#3\,$^{\mathrm{#2}}$%
    \fi
  \fi
  \par \egroup}
\def\address@fmt@init{%
  \par                                % Start new paragraph
   \vskip 6\p@ \@plus 3\p@ \@minus 1.5pt}

\newtoks\email@text
\newtoks\url@text
%\newtoks\t@glob@notes
\newtoks\t@loc@notes              % List of notes for one element
%\newcount\note@cnt                % Number of notes per element
\newcount\prev@elem \prev@elem=0  % Variables to keep track of
\newcount\cur@elem  \cur@elem=0   % types of elements that are processed

\newcounter{author}               % Author counter
\newcount\n@author                % Total number of authors
\def\n@author@{1}                  % idem, read from .aux file
\newcounter{collab}               % Collaboration counter
\newcount\n@collab                % Total number of collaborations
\def\n@collab@{}                  % idem, read from .aux file
\newcounter{address}              % Address counter
\def\theHaddress{\arabic{address}}% for hyperref
\newcount\note@cnt                % Number of notes per element

\chardef\e@title=1
\chardef\e@subtitle=1
\chardef\e@author=2
\chardef\e@collab=3
\chardef\e@address=4


%% The following is stolen from
%% ftp://tug.ctan.org/pub/tex-archive/macros/latex/contrib/tugboat/tugboat.dtx 

\newtoks\abstract@toks  \abstract@toks{}
\let\if@abstract\iffalse
\def\abstract{%
  \if@madetitle
    \ClassWarning{abstract environment after \string\maketitle}
  \fi
  \def\@abstract@{abstract}%
  \ifx\@currenvir\@abstract@
  \else
    \ClassError{\string\abstract\space is illegal:%
      \MessageBreak
      use \string\begin{\@abstract@} instead}%
      {\@abstract@\space may only be used as an environment}
  \fi
  \global\let\if@abstract\iftrue
  {\ifnum0=`}\fi
  \@abstract@getbody}
\let\endabstract\relax
\long\def\@abstract@getbody#1\end{%
  \global\abstract@toks\expandafter{\the\abstract@toks#1}%
  \@abstract@findend}
\def\@abstract@findend#1{%
  \def\@tempa{#1}%
  \ifx\@tempa\@abstract@
    \expandafter\@abstract@end
  \else
    \def\@tempb{document}%
    \ifx\@tempa\@tempb
      \ClassError{\string\begin{\@abstract@}
          ended by \string\end{\@tempb}}%
        {You've forgotten \string\end{\@abstract@}}
    \else
       \global\abstract@toks\expandafter{\the\abstract@toks\end{#1}}%
       \expandafter\expandafter\expandafter\@abstract@getbody
    \fi
  \fi}
\def\@abstract@end{\ifnum0=`{\fi}%
  \expandafter\end\expandafter{\@abstract@}}

\newtoks\keywords@toks  \keywords@toks{}
\let\if@keywords\iffalse
\def\keywords{%
  \if@madetitle
    \ClassWarning{keywords environment after \string\maketitle}
  \fi
  \def\@keywords@{keywords}%
  \ifx\@currenvir\@keywords@
  \else
    \ClassError{\string\keywords\space is illegal:%
      \MessageBreak
      use \string\begin{\@keywords@} instead}%
      {\@keywords@\space may only be used as an environment}
  \fi
  \global\let\if@keywords\iftrue
  {\ifnum0=`}\fi
  \@keywords@getbody}
\let\endkeywords\relax
\long\def\@keywords@getbody#1\end{%
  \global\keywords@toks\expandafter{\the\keywords@toks#1}%
  \@keywords@findend}
\def\@keywords@findend#1{%
  \def\@tempa{#1}%
  \ifx\@tempa\@keywords@
    \expandafter\@keywords@end
  \else
    \def\@tempb{document}%
    \ifx\@tempa\@tempb
      \ClassError{\string\begin{\@keywords@}
          ended by \string\end{\@tempb}}%
        {You've forgotten \string\end{\@keywords@}}
    \else
       \global\keywords@toks\expandafter{\the\keywords@toks\end{#1}}%
       \expandafter\expandafter\expandafter\@keywords@getbody
    \fi
  \fi}
\def\@keywords@end{\ifnum0=`{\fi}%
  \expandafter\end\expandafter{\@keywords@}}
