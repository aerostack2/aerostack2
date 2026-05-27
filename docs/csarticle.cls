%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%   FILE: csarticle.cls
%%   PURPOSE: CORESENSE Article
%%   LOADS: article.cls (Standard LaTeX class)
%%   PROGRAMMED: Ricardo Sanz, Oct. 2022
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

\NeedsTeXFormat{LaTeX2e}[1995/12/01]
\ProvidesClass{csarticle}[2022/10/20 v1.1 CORESENSE article LaTeX class]

\RequirePackage[xetex,table,dvipsnames,hyperref]{xcolor}

\definecolor{CSGreen}{HTML}{006A63}
\definecolor{CSRed}{HTML}{BF002A}
\definecolor{CSMaroon}{HTML}{512977}
\definecolor{CSOrange}{HTML}{EAB82D}

\newcommand{\headcolor}{\color{CSGreen}}
%\newcommand{\@header}{\headcolor\rule{1mm}{3cm}}
\newcommand{\@header}{\includegraphics[width=210mm]{CSheader.png}}
\newcommand{\header}[1]{\renewcommand{\@header}{\includegraphics[width=210mm]{#1}}}

\DeclareOption{green}{\renewcommand{\headcolor}{\color{CSGreen}}}
\DeclareOption{maroon}{\renewcommand{\headcolor}{\color{CSMaroon}}}
\DeclareOption{red}{\renewcommand{\headcolor}{\color{CSRed}}}
\DeclareOption{orange}{\renewcommand{\headcolor}{\color{CSOrange}}}

%\DeclareOption{noheader}{\renewcommand{\@header}{\headcolor\rule{1mm}{3cm}}}
\DeclareOption{noheader}{\renewcommand{\@header}{}}
\DeclareOption{header}{\header{CSheader.png}}

\DeclareOption*{\PassOptionsToClass{\CurrentOption}{article}}
\ProcessOptions\relax

% Base class -----------------------------------------------------

\LoadClass[twoside,a4paper,11pt]{article}

\usepackage{cscommon}

%-- Packages ----------------------------------------------------

\usepackage[export]{adjustbox}
\usepackage{calc}             
\usepackage{fancyhdr}
\usepackage{url} 

% Section Titles -------------------------------------------------

\newfontfamily\headingfont{Montserrat Bold}
\newfontfamily\displayfont{Montserrat Black}

\usepackage{titlesec}
\titleformat*{\section}{\fontsize{16}{16}\selectfont\headingfont}
%\titleformat*{\section}{\fontsize{16}{16}\selectfont\bfseries}
\titleformat*{\subsection}{\fontsize{14}{14}\selectfont\headingfont}
\titleformat*{\subsubsection}{\fontsize{12}{12}\selectfont\itshape}

%\titleformat*{\section}{\Large\bfseries}
%\titleformat*{\subsection}{\large\bfseries}
%\titleformat*{\subsubsection}{\itshape}

\titlespacing*{\section}{0pt}{20pt}{0pt}
\titlespacing*{\subsection}{0pt}{10pt}{0pt}
\titlespacing*{\subsubsection}{0pt}{0pt}{0pt}


%\titlespacing*{\section}{0pt}{2ex plus 0.5ex minus .2ex}{1ex plus .2ex}
%\titlespacing*{\subsection}{0pt}{1.5ex plus 0.5ex minus .2ex}{1ex plus .2ex}

%% Reference variable and command definition ------------------------------------

\newcommand{\@subtitle}{}
\newcommand{\subtitle}[1]{\renewcommand{\@subtitle}{#1}}

\newcommand{\@docyear}{}
\newcommand{\docyear}[1]{\renewcommand{\@docyear}{#1}}

\newcommand{\@shortauthor}{\@author}
\newcommand{\shortauthor}[1]{\renewcommand{\@shortauthor}{#1}}

\newcommand{\@shorttitle}{\@title}
\newcommand{\shorttitle}[1]{\renewcommand{\@shorttitle}{#1}}

\newcommand{\@straighttitle}{\@title}
\newcommand{\straighttitle}[1]{\renewcommand{\@straighttitle}{#1}}

\newcommand{\@reference}{}
\newcommand{\reference}[1]{\renewcommand{\@reference}{#1}}

\newcommand{\@version}{0.0}
\newcommand{\version}[1]{\renewcommand{\@version}{#1}}

\newcommand{\@status}{}
\newcommand{\status}[1]{\renewcommand{\@status}{#1}}

\newcommand{\@URL}{http://www.coresense.eu/doc/\@reference.pdf}
\newcommand{\URL}[1]{\renewcommand{\@URL}{#1}}

\newcommand{\@abstract}{}
\renewcommand{\abstract}[1]{\renewcommand{\@abstract}{#1}}

\newcommand{\@keywords}{}
\newcommand{\keywords}[1]{\renewcommand{\@keywords}{#1}}

\newcommand{\@acknowledgements}{}
\newcommand{\acknowledgements}[1]{\renewcommand{\@acknowledgements}{#1}}

\newcommand{\@email}{}
\newcommand{\email}[1]{\renewcommand{\@email}{#1}}

\newcommand{\@project}{CORESENSE}
\newcommand{\project}[1]{\renewcommand{\@project}{#1}}

%\newcommand{\@projectname}{CORESENSE.EU}
%\newcommand{\projectname}[1]{\renewcommand{\@projectname}{#1}}
          
\newcommand{\@logo}{coresense-logo-reduced.png}
\newcommand{\logo}[1]{\renewcommand{\@logo}{#1}}
          
\newcommand{\printauthor}{\@author}
\newcommand{\printtitle}{\@title}
\newcommand{\printemail}{\@email}
\newcommand{\printurl}{\@URL}
\newcommand{\printreference}{\@reference\ v \@version}

                                    
%% Title page ------------------------------------------------------

\renewcommand\maketitle{\par
  \newpage
  \null
  \begin{picture}(0,0)
  \put(-75,60){\headcolor\rule{21.3cm}{3cm}}
  \put(-72,70){\@header}
%  \put(-150,70){\includegraphics[width=55cm]{\@header}}
  \put(-20,25){\href{http://www.coresense.eu}{\includegraphics[height=3cm]{\@logo}}}
%  \put(380,66){\Huge \bfseries \color{white}\displayfont \@reference}
  \put(230,72){\parbox{8cm}{\hfill\Huge\bfseries\color{white}\displayfont \@reference}}
  \end{picture}
  \small {\href{http://www.coresense.eu}{\color{white}\@project}}\\[2cm]
  {\huge \bfseries \displayfont \@title \par}%
  {\ifx\@subtitle\@empty
   \else
      {\Large \@subtitle \par} 
  \fi}
  {\Large \textit\@author \par}
  {\large \@reference\ v \@version\ --- \@date\ \par}
  \par
  \vskip 1cm}


%-- Versions  --------------------------------------------

\newcommand{\documentversions}{
	\section*{Document Versions}
}

\newcommand{\versionitem}[4]{
	\begin{tabular}[t]{R{1.5cm}L{14cm}} 
		\bkx \textbf{Version} & \bkx  \textbf{#1} | \textbf{#4} \\
		\emph{Changes}   & #2  \\
		\emph{Authors}   & #3 \\%[4mm]
	\end{tabular}
	\newline
}

% --- Bibliography -------------------------------------------------

\renewenvironment{thebibliography}[1]
     {\section*{\refname}%
      \@mkboth{\refname}{\refname}%
      \list{\@biblabel{\@arabic\c@enumiv}}%
           {\settowidth\labelwidth{\@biblabel{#1}}%
            \leftmargin\labelwidth
            \advance\leftmargin\labelsep
            \@openbib@code
            \usecounter{enumiv}%
            \let\p@enumiv\@empty
            \renewcommand\theenumiv{\@arabic\c@enumiv}}%
      \sloppy
      \clubpenalty4000
      \@clubpenalty \clubpenalty
      \widowpenalty4000%
      \sfcode`\.\@m}

%-- Finalcover -----------------------------------------------

\newcommand{\finalcover}{
\newpage
%\oddsidemargin=1.5 cm
\thispagestyle{empty}

{\scriptsize The \textbf{CoreSense Project} is a research project funded by the European Commission Horizon Europe programme though grant \#101070254. Views and opinions expressed in this document are however those of the author(s) only and do not necessarily reflect those of the European Commission or the Horizon Europe programme. Neither the European Union nor the granting authority can be held responsible for them.}

\href{http://www.horizoneurope.eu}{\includegraphics[height=15mm]{EU_funded_en.jpg}}

\vspace*{\stretch{2}}

\begin{tabular}{rl}
& \hspace{-5mm}\includegraphics[width=25mm]{coresense-logo-reduced.png}\\[5mm]
\hfill\textit{Title}:&\ \@straighttitle\\
\hfill\textit{Author}:&\ \@author\\
\hfill\textit{Date}:&\ \@date\\
\hfill\textit{Reference}:&\ \@reference\ v \@version\ \@status\\
\hfill\textit{URL}:&\ \@URL\\[5mm]
&{\Large \href{http://www.coresense.eu}{\texttt{coresense.eu}}}
\end{tabular}
}
%-- Page & Paragraph formatting ------------------------------

\setlength \paperheight {297mm}
\setlength \paperwidth  {210mm}
\setlength \textwidth {160mm}  
\setlength \textheight {230mm} 
\setlength \hoffset {-22mm}
\setlength \voffset {-30mm}
\setlength \oddsidemargin {22mm}
\setlength \evensidemargin {22mm}
\setlength \marginparwidth {11mm}
\setlength \topmargin {20mm}  %% margin from the top to header
\setlength \footskip {10mm}   %% Footer distance from text


\setlength \parindent {0em}           
\setlength \parskip {2mm}
\setlength \headheight {14pt}           
\setlength \headsep {10mm}
\setlength \partopsep {0pt}

%-- Headers & Footers -------------------------------------------

\pagestyle{fancy}
\fancyhf{}

\fancyhead[LE]{\textit {{\small\@straighttitle}}}
\fancyhead[RO]{\textit {{\small\nouppercase{\leftmark}}}}
\fancyfoot[LE]{{\bfseries\thepage} | \@reference\ | v\@version\ | \href{http://www.coresense.eu}{\@project} } 
\fancyfoot[RO]{\href{http://www.coresense.eu}{\@project} | \@reference\ | \@version\ | \bfseries\thepage } 

\renewcommand{\footrulewidth}{0pt}
\renewcommand{\headrulewidth}{0pt}
\renewcommand{\footskip}{50pt}

%-- Abstract paragraph -----------------------------

\newcommand{\printabstract}{
\noindent\textbf{Abstract}: \@abstract
}

%-- Keyword paragraph -----------------------------

\newcommand{\printkeywords}{
\noindent\textbf{Keywords}: \@keywords
}
%-- Float Placement Restrictions -----------------------------

\renewcommand{\textfraction}{0.05}
\renewcommand{\topfraction}{0.95}
\renewcommand{\bottomfraction}{0.95}

