function [F,G]=sindiophantine(A,B,C,varargin)
%��С��������������������ר�ý��ж���ͼ���̵����C=A*F+Z^-d*G �� C=A*F+B*G
%ȷ��ϵͳ�״�
    if (nargin==3) %����������Ĳ�����z^-d
        na=length(A)-1;
        nb=length(B)-1;
        nc=length(C)-1;%na,nb,ncΪ����ʽA B C�״�,C���Ѿ�����P
        nf=nb-1;
        ng=max([na-1,nc-nb]);%���ݽ״��������������ʽ�״�
        nmax=max([nc,nf+na,ng+nb]);

        %��������
        F=sym('x',[1 nf+1]);
        G=sym('y',[1 ng+1]);
        p1=convar(A,F);
        p2=convar(B,G);
        %��������������ͬ�״�
        if(length(p1)<nmax+1)
            p1(1,nmax+1)=0;
        end
        if(length(p2)<nmax+1)
            p2(1,nmax+1)=0;
        end
        if(length(C)<nmax+1)
            C(1,nmax+1)=0;
        end
        %����p1 p2 C��ⷽ��
        epns=p1+p2-C;
        vars=[F,G];
        H=sym('h',[1 nf+1+ng+1]);
        H=solve(epns,vars);
        H=structToarray(H);
        F=H(1,1:length(F));
        G=H(1,length(F)+1:length(F)+length(G));
    elseif (nargin==4) %����������İ���z^-d
        d=varargin{1};
        na=length(A)-1;
        nb=length(B)-1;
        nc=length(C)-1;%na,nb,ncΪ����ʽA B C�״�,C���Ѿ�����P
        nf=d-1;
        ng=max([na-1,nc-d]);%���ݽ״��������������ʽ�״�
        nmax=max([nc,nf+na,ng+d]);
        %��BתΪ����z-d��ʽ��������ⶪ��ͼ����
        B=zeros(1,nmax);
        B(1,d+1)=1;

        %��������
        F=sym('x',[1 nf+1]);
        G=sym('y',[1 ng+1]);
        p1=convar(A,F);
        p2=convar(B,G);
        %��������������ͬ�״�
        if(length(p1)<nmax+1)
            p1(1,nmax+1)=0;
        end
        if(length(p2)<nmax+1)
            p2(1,nmax+1)=0;
        end
        if(length(C)<nmax+1)
            C(1,nmax+1)=0;
        end
        %����p1 p2 C��ⷽ��
        epns=p1+p2-C;
        vars=[F,G];
        H=sym('h',[1 nf+1+ng+1]);
        H=solve(epns,vars);
        H=structToarray(H);
        F=H(1,1:length(F));
        G=H(1,length(F)+1:length(F)+length(G));

    end




