/*!
 *****************************************************************************
 *
 *  Copyright © 2017-2018 Vesa.Mount. All Rights Reserved.
 *
 * \file      my_macro.c
 * \author    Ouqichen
 * \version   1.0
 * \date      2018年3月24日
 * \brief     Source Insight 4.0 宏命令
 *
 *----------------------------------------------------------------------------
 * \attention
 *
 *
 *****************************************************************************
 */

/*****************************************************************************
 change history: 
    1.date  : 2018年3月24日
      author: Ouqichen
      change: create file

*****************************************************************************/

/*!
 * \fn     strstr
 * \brief  比较str1中是否包含str2
 *          
 * \param  [in] str1   源字符串
 * \param  [in] str2   子字符串
 * 
 * \retval 
 */

/************************************************
 change history: 
    1.date  : 2018年3月24日
      author: Ouqichen
      change: create function

************************************************/
macro strstr(str1,str2)
{
    i = 0
    j = 0
    len1 = strlen(str1)
    len2 = strlen(str2)
    if((len1 == 0) || (len2 == 0))
    {
        return 0xffffffff
    }
    while( i < len1)
    {
        if(str1[i] == str2[j])
        {
            while(j < len2)
            {
                j = j + 1
                if(str1[i+j] != str2[j]) 
                {
                    break
                }
            }     
            if(j == len2)
            {
                return i
            }
            j = 0
        }
        i = i + 1      
    }  
    return 0xffffffff
}


/*!
 * \fn     CommentContent
 * \brief  自动排版函数，列小于75
 *          
 * \param  [in] hbuf        #
 * \param  [in] ln          #
 * \param  [in] szPreStr    #
 * \param  [in] szContent   #
 * \param  [in] isEnd       #
 * 
 * \retval 
 */

/************************************************
 change history: 
    1.date  : 2018年3月24日
      author: Ouqichen
      change: create function

************************************************/
macro CommentContent (hbuf,ln,szPreStr,szContent,isEnd)
{
    szLeftBlank = szPreStr
    iLen = strlen(szPreStr)
    k = 0
    while(k < iLen)
    {
        szLeftBlank[k] = " ";
        k = k + 1;
    }

    hNewBuf = newbuf("clip")
    if(hNewBuf == hNil)
        return       
    SetCurrentBuf(hNewBuf)
    PasteBufLine (hNewBuf, 0)
    lnMax = GetBufLineCount( hNewBuf )
    szTmp = TrimString(szContent)

    //判断如果剪贴板是0行时对于有些版本会有问题，要排除掉
    if(lnMax != 0)
    {
        szLine = GetBufLine(hNewBuf , 0)
	    ret = strstr(szLine,szTmp)
	    if(ret == 0)
	    {
	        /*如果输入窗输入的内容是剪贴板的一部分说明是剪贴过来的取剪贴板中的内
	          容*/
	        szContent = TrimString(szLine)
	    }
	    else
	    {
	        lnMax = 1
	    }	    
    }
    else
    {
        lnMax = 1
    }    
    szRet = ""
    nIdx = 0
    while ( nIdx < lnMax) 
    {
        if(nIdx != 0)
        {
            szLine = GetBufLine(hNewBuf , nIdx)
            szContent = TrimLeft(szLine)
            szPreStr = szLeftBlank
        }
        iLen = strlen (szContent)
        szTmp = cat(szPreStr,"#");
        if( (iLen == 0) && (nIdx == (lnMax - 1))
        {
            InsBufLine(hbuf, ln, "@szTmp@")
        }
        else
        {
            i = 0
            //以每行100个字符处理
            while  (iLen - i > 75 - k )
            {
                j = 0
                while(j < 75 - k)
                {
                    iNum = szContent[i + j]
                    if(AsciiFromChar(iNum) < 32 )// 测试发现AsciiFromChar该函数对于汉字  都是返回小于32的值
                    {
                       j = j + 3// Utf-8对于一般汉字都是3字节，比较生僻的字可能会引起排版问题
                    }
                    else
                    {
                       j = j + 1
                    }
                    //if( (j > 96 - k) && (szContent[i + j] == " ") )
                    //{
                    //   break
                    //}
                }
                
                if((szContent[i + j] != " " ))
                {
                    n = 0;
                    iNum = szContent[i + j + n]
                    //如果是中文字符只能成对处理
                    while( (iNum != " " ) && (AsciiFromChar(iNum)  > 32))
                    {
                        n = n + 1
                        if((n >= 3) ||(i + j + n >= iLen))
                             break;
                        iNum = szContent[i + j + n]
                    }
                    if(n < 3)
                    {
                        //分段后只有小于3个的字符留在下段则将其以上去
                        j = j + n 
                        sz1 = strmid(szContent,i,i+j)
                        sz1 = cat(szPreStr,sz1)                
                    }
                    else
                    {
                        //大于3个字符的加连字符分段
                        sz1 = strmid(szContent,i,i+j)
                        sz1 = cat(szPreStr,sz1)
                        if(sz1[strlen(sz1)-1] != "-")
                        {
                           sz1 = cat(sz1,"-")                
                        }
                    }
                }
                else
                {
                    sz1 = strmid(szContent,i,i+j)
                    sz1 = cat(szPreStr,sz1)
                }
                InsBufLine(hbuf, ln, "@sz1@")
                ln = ln + 1
                szPreStr = szLeftBlank
                i = i + j
                while(szContent[i] == " ")
                {
                    i = i + 1
                }
            }
            sz1 = strmid(szContent, i, iLen)
            sz1 = cat(szPreStr, sz1) 

            if((isEnd == 1) && (nIdx == (lnMax - 1))
            {
                sz1 = cat(sz1, " */")
            }
            InsBufLine(hbuf, ln, "@sz1@")
        }
        ln = ln + 1
        nIdx = nIdx + 1
    }
    closebuf(hNewBuf)
    return ln - 1
}

/*!
 * \fn     SkipCommentFromString
 * \brief  忽略注释函数
 *          
 * \param  [in] szLine         #
 * \param  [in] isCommentEnd   #
 * 
 * \retval 
 */

/************************************************
 change history: 
    1.date  : 2018年3月24日
      author: Ouqichen
      change: create function

************************************************/
macro SkipCommentFromString(szLine,isCommentEnd)
{
    RetVal = ""
    fIsEnd = 1
    nLen = strlen(szLine)
    nIdx = 0
    while(nIdx < nLen )
    {
        //如果当前行开始还是被注释，或遇到了注释开始的变标记，注释内容改为空格?
        if( (isCommentEnd == 0) || (szLine[nIdx] == "/" && szLine[nIdx+1] == "*"))
        {
            fIsEnd = 0
            while(nIdx < nLen )
            {
                if(szLine[nIdx] == "*" && szLine[nIdx+1] == "/")
                {
                    szLine[nIdx+1] = " "
                    szLine[nIdx] = " " 
                    nIdx = nIdx + 1 
                    fIsEnd  = 1
                    isCommentEnd = 1
                    break
                }
                szLine[nIdx] = " "
                
                //如果是倒数第二个则最后一个也肯定是在注释内
//                if(nIdx == nLen -2 )
//                {
//                    szLine[nIdx + 1] = " "
//                }
                nIdx = nIdx + 1 
            }    
            
            //如果已经到了行尾终止搜索
            if(nIdx == nLen)
            {
                break
            }
        }
        
        //如果遇到的是//来注释的说明后面都为注释
        if(szLine[nIdx] == "/" && szLine[nIdx+1] == "/")
        {
            szLine = strmid(szLine,0,nIdx)
            break
        }
        nIdx = nIdx + 1                
    }
    RetVal.szContent = szLine;
    RetVal.fIsEnd = fIsEnd
    return RetVal
}

/*!
 * \fn     GetFunctionDef
 * \brief  获得函数参数
 *          
 * \param  [in] hbuf     #
 * \param  [in] symbol   #
 * 
 * \retval 
 */

/************************************************
 change history: 
    1.date  : 2018年3月24日
      author: Ouqichen
      change: create function

************************************************/
macro GetFunctionDef(hbuf,symbol)
{
    ln = symbol.lnName
    szFunc = ""
    if(strlen(symbol) == 0)
    {
       return szFunc
    }
    fIsEnd = 1
//    msg(symbol)
    while(ln < symbol.lnLim)
    {
        szLine = GetBufLine (hbuf, ln)
        //去掉被注释掉的内容
        RetVal = SkipCommentFromString(szLine,fIsEnd)
		szLine = RetVal.szContent
		szLine = TrimString(szLine)
		fIsEnd = RetVal.fIsEnd
        //如果是{表示函数参数头结束了
        ret = strstr(szLine,"{")        
        if(ret != 0xffffffff)
        {
            szLine = strmid(szLine,0,ret)
            szFunc = cat(szFunc,szLine)
            break
        }
        szFunc = cat(szFunc,szLine)        
        ln = ln + 1
    }
    return szFunc
}

/*!
 * \fn     TrimLeft
 * \brief  去除字符串左边空格
 *          
 * \param  [in] szLine   #
 * 
 * \retval 
 */

/************************************************
 change history: 
    1.date  : 2018年3月24日
      author: Ouqichen
      change: create function

************************************************/
macro TrimLeft(szLine)
{
    nLen = strlen(szLine)
    if(nLen == 0)
    {
        return szLine
    }
    nIdx = 0
    while( nIdx < nLen )
    {
        if( ( szLine[nIdx] != " ") && (szLine[nIdx] != "\t") )
        {
            break
        }
        nIdx = nIdx + 1
    }
    return strmid(szLine,nIdx,nLen)
}

/*!
 * \fn     TrimRight
 * \brief  去除字符串右边空格
 *          
 * \param  [in] szLine   #
 * 
 * \retval 
 */

/************************************************
 change history: 
    1.date  : 2018年3月24日
      author: Ouqichen
      change: create function

************************************************/
macro TrimRight(szLine)
{
    nLen = strlen(szLine)
    if(nLen == 0)
    {
        return szLine
    }
    nIdx = nLen
    while( nIdx > 0 )
    {
        nIdx = nIdx - 1
        if( ( szLine[nIdx] != " ") && (szLine[nIdx] != "\t") )
        {
            break
        }
    }
    return strmid(szLine,0,nIdx+1)
}

/*!
 * \fn     TrimString
 * \brief  去除字符串两边空格
 *          
 * \param  [in] szLine   #
 * 
 * \retval 
 */

/************************************************
 change history: 
    1.date  : 2018年3月24日
      author: Ouqichen
      change: create function

************************************************/
macro TrimString(szLine)
{
    szLine = TrimLeft(szLine)
    szLIne = TrimRight(szLine)
    return szLine
}

/*!
 * \fn     GetWordFromString
 * \brief  分离函数参数
 *          
 * \param  [in] hbuf          #
 * \param  [in] szLine        #
 * \param  [in] nBeg          #
 * \param  [in] nEnd          #
 * \param  [in] chBeg         #
 * \param  [in] chSeparator   #
 * \param  [in] chEnd         #
 * 
 * \retval 
 */

/************************************************
 change history: 
    1.date  : 2018年3月24日
      author: Ouqichen
      change: create function

************************************************/
macro GetWordFromString(hbuf,szLine,nBeg,nEnd,chBeg,chSeparator,chEnd)
{
    if((nEnd > strlen(szLine) || (nBeg > nEnd))
    {
        return 0
    }
    nMaxLen = 0
    nIdx = nBeg
    //先定位到开始字符标记处
    while(nIdx < nEnd)
    {
        if(szLine[nIdx] == chBeg)
        {
            break
        }
        nIdx = nIdx + 1
    }
    nBegWord = nIdx + 1
    
    //用于检测chBeg和chEnd的配对情况
    iCount = 0
    
    nEndWord = 0
    //以分隔符为标记进行搜索
    while(nIdx < nEnd)
    {
        if(szLine[nIdx] == chSeparator)
        {
           szWord = strmid(szLine,nBegWord,nIdx)
           szWord = TrimString(szWord)
           nLen = strlen(szWord)
           if(nMaxLen < nLen)
           {
               nMaxLen = nLen
           }
           AppendBufLine(hbuf,szWord)
           nBegWord = nIdx + 1
        }
        if(szLine[nIdx] == chBeg)
        {
            iCount = iCount + 1
        }
        if(szLine[nIdx] == chEnd)
        {
            iCount = iCount - 1
            nEndWord = nIdx
            if( iCount == 0 )
            {
                break
            }
        }
        nIdx = nIdx + 1
    }
    if(nEndWord > nBegWord)
    {
        szWord = strmid(szLine,nBegWord,nEndWord)
        szWord = TrimString(szWord)
        nLen = strlen(szWord)
        if(nMaxLen < nLen)
        {
            nMaxLen = nLen
        }
        AppendBufLine(hbuf,szWord)
    }
    return nMaxLen
}

/*!
 * \fn     CreateBlankString
 * \brief  创建空白字符串
 *          
 * \param  [in] nBlankCount   #
 * 
 * \retval 
 */

/************************************************
 change history: 
    1.date  : 2018年3月24日
      author: Ouqichen
      change: create function

************************************************/
macro CreateBlankString(nBlankCount)
{
    szBlank=""
    nIdx = 0
    while(nIdx < nBlankCount)
    {
        szBlank = cat(szBlank," ")
        nIdx = nIdx + 1
    }
    return szBlank
}

/*!
 * \fn     HeadComment
 * \brief  函数头注释功能
 *          
 * \param  [in] hbuf       #
 * \param  [in] ln         #
 * \param  [in] szFunc     #
 * \param  [in] szMyName   #
 * 
 * \retval 
 */

/************************************************
 change history: 
    1.date  : 2018年3月24日
      author: Ouqichen
      change: create function

************************************************/
macro HeadComment(hbuf, ln, szFunc, szMyName)
{
	szContent = Ask("请输入函数功能描述的内容")
    if(strlen(szContent) == 0)
    	stop
    symbol = GetSymbolLocationFromLn(hbuf, ln)
    if(strlen(symbol) > 0)
    {
        hTmpBuf = NewBuf("Tempbuf")
        if(hTmpBuf == hNil)
        {
            stop
        }
        //将文件参数头整理成一行并去掉了注释
        szLine = GetFunctionDef(hbuf,symbol)            
        iBegin = symbol.ichName 
        //取出返回值定义
        szTemp = strmid(szLine,0,iBegin)
        szRet = TrimString(szTemp)
        
        if(toupper(szRet) == "MACRO")
        {
            //对于宏返回值特殊处理
            szRet = ""
        }
        //从函数头分离出函数参数
        nMaxParamSize = GetWordFromString(hTmpBuf,szLine,iBegin,strlen(szLine),"(",",",")")
        lnMax = GetBufLineCount(hTmpBuf)
        ln = symbol.lnFirst
        SetBufIns(hbuf, ln, 0)
    }
	oldln = ln  
    InsBufLine(hbuf, ln, "/*!") 
    if( strlen(szFunc)>0 )
    {
        InsBufLine(hbuf, ln+1, " * \\fn     @szFunc@") 
    }
    else
    {
        InsBufLine(hbuf, ln+1, " * \\fn     #")
    }
    
    InsBufLine(hbuf, ln+2, " * \\brief  ") 
    InsBufLine(hbuf, ln+3, " *          ") 
    ln = ln + 1
    szIns = " * \\param  [in] "
    
    i = 0
    while ( i < lnMax) 
    {
        szTmp = GetBufLine(hTmpBuf, i)
        nLen = strlen(szTmp);
        szBlank = CreateBlankString(nMaxParamSize - nLen + 2)
        szTmp = cat(szTmp,szBlank)
        ln = ln + 1
        szTmp = cat(szIns,szTmp)
        InsBufLine(hbuf, ln + 2, "@szTmp@ #") 
        i = i + 1
    }    
    closebuf(hTmpBuf)
        
    SysTime = GetSysTime(1)
    szTime = SysTime.Date
    
    InsBufLine(hbuf, ln + 3, " * ")
    InsBufLine(hbuf, ln + 4, " * \\retval @szRet@") 
    InsbufLIne(hbuf, ln + 5, " */");
//  InsbufLIne(hbuf, ln + 6, "");
/*
    InsBufLine(hbuf, ln + 7, "/************************************************")
    InsBufLine(hbuf, ln + 8, " change history: ")
    InsBufLine(hbuf, ln + 9, "    1.date  : @szTime@")

    if( strlen(szMyName) > 0 )
    {
       InsBufLine(hbuf, ln + 10, "      author: @szMyName@")
    }
    else
    {
       InsBufLine(hbuf, ln + 10, "      author: #")
    }
    InsBufLine(hbuf, ln + 11, "      change: create function")    
    InsBufLine(hbuf, ln + 12, "")    
    */
    //InsBufLine(hbuf, ln + 13, "************************************************/"

    
    DelBufLine(hbuf, oldln + 2)

    //显示输入的功能描述内容
    CommentContent(hbuf, oldln + 2, " * \\brief  ", szContent, 0)
}

/*!
 * \fn     O_InsertHeader
 * \brief  插入函数头
 *          
 * 
 * \retval 
 */

/************************************************
 change history: 
    1.date  : 2018年3月24日
      author: Ouqichen
      change: create function

************************************************/
macro O_InsertHeader()
{
    hwnd = GetCurrentWnd()
    if (hwnd == 0)
        stop
    sel = GetWndSel(hwnd)
    ln = sel.lnFirst
    hbuf = GetWndBuf(hwnd)
    szMyName = getreg(MYNAME)
    if(strlen( szMyName ) == 0)
    {
        szMyName = Ask("请输入你的名字：")
        setreg(MYNAME, szMyName)
    }
    lnMax = GetBufLineCount(hbuf)
    if(ln != lnMax)
    {
        szNextLine = GetBufLine(hbuf,ln)
        if( (strstr(szNextLine,"(") != 0xffffffff))
        {
            symbol = GetCurSymbol()
            if(strlen(symbol) != 0)
            {  
				HeadComment(hbuf, ln, symbol, szMyName) 
                return
            }
        }
    }
    msg("无法生成函数注释，请将光标置于需要生成的函数上")
}

/*!
 * \fn     InsertHistoryContent
 * \brief  插入更新内容
 *          
 * \param  [in] hbuf            #
 * \param  [in] ln              #
 * \param  [in] iHostoryCount   #
 * 
 * \retval 
 */

/************************************************
 change history: 
    1.date  : 2018年3月24日
      author: Ouqichen
      change: create function

************************************************/
macro  InsertHistoryContent(hbuf,ln,iHostoryCount)
{
	SysTime = GetSysTime(1)
	szTime = SysTime.Date
	szMyName = getreg(MYNAME)

	szContent = Ask("请输入修改的内容")
	if(strlen(szContent) == 0)
		stop
	InsBufLine(hbuf, ln, "")
	InsBufLine(hbuf, ln + 1, "    @iHostoryCount@.date  : @szTime@")

	if( strlen(szMyName) > 0 )
	{
		InsBufLine(hbuf, ln + 2, "      author: @szMyName@")
	}
	else
	{
		InsBufLine(hbuf, ln + 2, "      author: #")
	}
	CommentContent(hbuf,ln + 3,"      change: ",szContent,0)
}

/*!
 * \fn     O_InsertHistory
 * \brief  插入更新
 *          
 * 
 * \retval 
 */

/************************************************
 change history: 
    1.date  : 2018年3月24日
      author: Ouqichen
      change: create function

************************************************/
macro O_InsertHistory()
{
	flag_valid = 0 //是否具备更新条件标志
	hwnd = GetCurrentWnd()
	if (hwnd == 0)
        stop
    sel = GetWndSel(hwnd)
    ln = sel.lnFirst
    hbuf = GetWndBuf(hwnd)
    
    szMyName = getreg(MYNAME)
    if(strlen( szMyName ) == 0)
    {
        szMyName = Ask("请输入你的名字：")
        setreg(MYNAME, szMyName)
    }

	szCurLine = GetBufLine(hbuf, ln + 1) 
    if((strstr(szCurLine,"****/") != 0xffffffff)
    {
		ln = ln
		flag_valid = 1
    }
    
    szCurLine = GetBufLine(hbuf, ln)
    if((strstr(szCurLine,"(") != 0xffffffff))
    {
		ln = ln - 2
		flag_valid = 1
    }
    
    iHistoryCount = 1
    i = 0
    while(ln - i > 0)
    {
        szCurLine = GetBufLine(hbuf, ln-i)
        iBeg = strstr(szCurLine,"date  :")
        if(iBeg != 0xffffffff)
        {
            iHistoryCount = iHistoryCount + 1
            i = i + 1
            continue
        }
        iBeg = strstr(szCurLine,"history")
        if(iBeg != 0xffffffff)
        {
            break
        }
        iBeg = strstr(szCurLine,"/*********")
        if( iBeg != 0xffffffff )
        {
            break
        }
        iBeg = strstr(szCurLine,"}")
        if( iBeg != 0xffffffff )
        {
            break
        }
        i = i + 1
    }
    if(iHistoryCount == 1)
    {
    	O_InsertHeader()
    	stop
    }
    if(flag_valid == 1)
    {
    	InsertHistoryContent(hbuf,ln,iHistoryCount)
    }else
    {
    	msg("无法更新，请将光标置于函数名或历史记录的最后一行")
    	stop
    }
  
}

/*!
 * \fn     GetFileName
 * \brief  获得文件名
 *          
 * \param  [in] sz   #
 * 
 * \retval 
 */

/************************************************
 change history: 
    1.date  : 2018年3月24日
      author: Ouqichen
      change: create function

************************************************/
macro GetFileName(sz)
{
    i = 1
    szName = sz
    iLen = strlen(sz)
    if(iLen == 0)
      return ""
    while( i <= iLen)
    {
      if(sz[iLen-i] == "\\")
      {
        szName = strmid(sz,iLen-i+1,iLen)
        break
      }
      i = i + 1
    }
    return szName
}

/*!
 * \fn     InsertFileHead
 * \brief  插入文件头
 *          
 * \param  [in] hbuf        #
 * \param  [in] ln          #
 * \param  [in] szName      #
 * \param  [in] szContent   #
 * \param  [in] szOwner     #
 * 
 * \retval 
 */

/************************************************
 change history: 
    1.date  : 2018年3月24日
      author: Ouqichen
      change: create function

************************************************/
macro InsertFileHead(hbuf, ln,szName,szContent,szOwner)
{
	if(strlen(szOwner) == 0)
	{
		szOwner = ASK("请输入版权所有者")
		SetReg ("OWNER", szOwner)		
	}
    InsBufLine(hbuf, ln + 0, "/*!")
    InsBufLine(hbuf, ln + 1, " *****************************************************************************")
    InsBufLine(hbuf, ln + 2, " *")
    InsBufLine(hbuf, ln + 3, " *  Copyright © 2017-2018 @szOwner@. All Rights Reserved.")
    InsBufLine(hbuf, ln + 4, " *")
    sz = GetFileName(GetBufName (hbuf))
    InsBufLine(hbuf, ln + 5, " * \\file      @sz@")
    InsBufLine(hbuf, ln + 6, " * \\author    @szName@")
    InsBufLine(hbuf, ln + 7, " * \\version   1.0")
    SysTime = GetSysTime(1)
    szTime = SysTime.Date
    InsBufLine(hbuf, ln + 8, " * \\date      @szTime@")
    InsBufLine(hbuf, ln + 9, " * \\brief     @szContent@")
    InsBufLine(hbuf, ln + 10, " *")
    InsBufLine(hbuf, ln + 11, " *----------------------------------------------------------------------------")
    InsBufLine(hbuf, ln + 12, " * \\attention")
    InsBufLine(hbuf, ln + 13, " *")
    InsBufLine(hbuf, ln + 14, " *")
    InsBufLine(hbuf, ln + 15, " *****************************************************************************")
	InsBufLine(hbuf, ln + 16, " */")

	InsbufLIne(hbuf, ln+17, "");

    InsBufLine(hbuf, ln+18, "/*****************************************************************************")
    InsBufLine(hbuf, ln+19, " change history: ")
    InsBufLine(hbuf, ln+20, "    1.date  : @szTime@")

    if( strlen(szMyName)>0 )
    {
       InsBufLine(hbuf, ln+21, "      author: @szName@")
    }
    else
    {
       InsBufLine(hbuf, ln+21, "      author: #")
    }
    InsBufLine(hbuf, ln+22, "      change: create file")    
    InsBufLine(hbuf, ln+23, "")    
    InsBufLine(hbuf, ln+24, "*****************************************************************************/"
    
    //if(strlen(szContent) != 0)
    //{
    //    return
    //} 

    if(strlen(szContent) == 0)	    //如果没有输入功能描述的话提示输入
	    szContent = Ask("请输入文件功能描述的内容")
	    
    DelBufLine(hbuf,ln +9)
    //自动排列显示功能描述
    CommentContent(hbuf,ln+9," * \\brief     ",szContent,0)

    
}

/*!
 * \fn     O_InsertFileHeader
 * \brief  插入文件头
 *          
 * 
 * \retval 
 */

/************************************************
 change history: 
    1.date  : 2018年3月24日
      author: Ouqichen
      change: create function

************************************************/
macro O_InsertFileHeader()
{
    hwnd = GetCurrentWnd()
    if (hwnd == 0)
        stop
    ln = 0
    hbuf = GetWndBuf(hwnd)
    szMyName = getreg(MYNAME)
    if(strlen( szMyName ) == 0)
    {
        szMyName = Ask("请输入你的名字：")
        setreg(MYNAME, szMyName)
    }
    SetBufIns(hbuf, 0, 0)
    szOwner = getreg(OWNER)
    InsertFileHead(hbuf, ln, szMyName, "", szOwner)
}

/*!
 * \fn     O_Configure
 * \brief  参数配置
 *          
 * 
 * \retval 
 */

/************************************************
 change history: 
    1.date  : 2018年3月24日
      author: Ouqichen
      change: create function

************************************************/
macro O_Configure()
{
	szName = ASK("请输入你的名字：");
    if(szName == "#")
    {
       SetReg ("MYNAME", "")
    }
    else
    {
       SetReg ("MYNAME", szName)
    }
    
    szOwner = ASK("请输入版权所有者");
    if(szOwner == "#")
    {
       SetReg ("OWNER", "")
    }
    else
    {
       SetReg ("OWNER", szOwner)
    }
}


/*!
 * \fn     QuickCommentLine
 * \brief  快速插入单行注释
 *          
 * \param  [in] szName   #
 * \param  [in] hbuf     #
 * \param  [in] sel      #
 * 
 * \retval 
 */

/************************************************
 change history: 
    1.date  : 2018年3月24日
      author: Ouqichen
      change: create function

************************************************/
macro QuickCommentLine(szName, hbuf, sel)
{
	szCurLine = GetBufLine(hbuf, sel.lnFirst);
	szContent = Ask("请输入注释的内容")
	if(strlen(szContent) == 0)
		stop
		
	DelBufLine(hbuf, sel.lnFirst)
	szCurLine = cat(szCurLine, " //!< ")
	szCurLine = cat(szCurLine, szContent)
	SysTime = GetSysTime(1)
    szTime = SysTime.Date
	InsBufLine(hbuf, sel.lnFirst, "@szCurLine@ By: @szName@ @szTime@")
}


/*!
 * \fn     O_QuickCommentLine
 * \brief  快速插入单行注释
 *          
 * 
 * \retval 
 */

/************************************************
 change history: 
    1.date  : 2018年3月24日
      author: Ouqichen
      change: create function

************************************************/
macro O_QuickCommentLine()
{
	hwnd = GetCurrentWnd()
    if (hwnd == 0)
        stop
    sel = GetWndSel(hwnd)
	hbuf = GetWndBuf(hwnd)
	
    szMyName = getreg(MYNAME)
    if(strlen( szMyName ) == 0)
    {
        szMyName = Ask("请输入你的名字：")
        setreg(MYNAME, szMyName)
    }
    QuickCommentLine(szMyName, hbuf, sel)

}

//快速插入多行注释
macro CommentContentLines (hbuf,ln,szPreStr,szContent)
{
    szLeftBlank = szPreStr
    k = strlen(szPreStr)
	DelBufLine(hbuf, ln)    
    InsBufLine(hbuf, ln, "@szLeftBlank@/*!")
    ln = ln + 1
	iLen = strlen(szContent)
	i = 0
	while  (iLen - i > 75 - k )
    {
        j = 0
        while(j < 75 - k)
        {
            iNum = szContent[i + j]
            if(AsciiFromChar(iNum) < 32 )// 测试发现AsciiFromChar该函数对于汉字  都是返回小于32的值
            {
               j = j + 3// Utf-8对于一般汉字都是3字节，比较生僻的字可能会引起排版问题
            }
            else
            {
               j = j + 1
            }
        }
        
        if((szContent[i + j] != " " ))
        {
            n = 0;
            iNum = szContent[i + j + n]
            //如果是中文字符只能成对处理
            while( (iNum != " " ) && (AsciiFromChar(iNum)  > 32))
            {
                n = n + 1
                if((n >= 3) ||(i + j + n >= iLen))
                     break;
                iNum = szContent[i + j + n]
            }
            if(n < 3)
            {
                //分段后只有小于3个的字符留在下段则将其以上去
                j = j + n 
                sz1 = strmid(szContent,i,i+j)        
            }
            else
            {
                //大于3个字符的加连字符分段
                sz1 = strmid(szContent,i,i+j)
                if(sz1[strlen(sz1)-1] != "-")
                {
                   sz1 = cat(sz1,"-")                
                }
            }
        }
        else
        {
            sz1 = strmid(szContent,i,i+j)
        }
        InsBufLine(hbuf, ln, "@szLeftBlank@ * @sz1@")
        ln = ln + 1
        i = i + j
        while(szContent[i] == " ")
        {
            i = i + 1
        }
    }
    sz1 = strmid(szContent, i, iLen)
    InsBufLine(hbuf, ln, "@szLeftBlank@ * @sz1@")
    ln = ln + 1
	InsBufLine(hbuf, ln, "@szLeftBlank@ */")
    return ln - 1
}


/*!
 * \fn     O_QuickCommentLines
 * \brief  快速插入多行注释
 *          
 * 
 * \retval 
 */

/************************************************
 change history: 
    1.date  : 2018年3月24日
      author: Ouqichen
      change: create function

************************************************/
macro O_QuickCommentLines()
{
	hwnd = GetCurrentWnd()
    if (hwnd == 0)
        stop
    sel = GetWndSel(hwnd)
	hbuf = GetWndBuf(hwnd)
	
    szMyName = getreg(MYNAME)
    if(strlen( szMyName ) == 0)
    {
        szMyName = Ask("请输入你的名字：")
        setreg(MYNAME, szMyName)
    }
    
    szCurLine = GetBufLine(hbuf, sel.lnFirst);
    i = 0
    while(i < strlen(szCurLine))//判断该行是否有非空格字符，如果有则退出本宏
    {
    	if(AsciiFromChar(szCurLine[i]) != 32 && AsciiFromChar(szCurLine[i]) != 9)
    	{
    		msg("多行注释仅支持空行")
    		stop
    	}
    	i = i + 1
    }
    szContent = Ask("请输入注释的内容")
    if(strlen(szContent) == 0)
		stop
	szCurLine = strmid(szCurLine,0,sel.ichFirst)
    CommentContentLines (hbuf, sel.lnFirst, szCurLine, szContent)
}


/*!
 * \fn     O_MultiLineComment
 * \brief  行注释
 *          
 * 
 * \retval 
 */

/************************************************
 change history: 
    1.date  : 2018年3月24日
      author: Ouqichen
      change: create function

************************************************/
macro O_MultiLineComment()
{
	hwnd = GetCurrentWnd()
    if (hwnd == 0)
        stop 
    sel = GetWndSel(hwnd)
	hbuf = GetWndBuf(hwnd)
	
    curLineNum = sel.lnFirst 
	
    while(curLineNum <= sel.lnLast)
    {
    	szCurLine = GetBufLine(hbuf, curLineNum)
    	if(szCurLine == "")
    	{
    		curLineNum = curLineNum + 1
    		continue
    	}
    	tmpLen = strlen(szCurLine)
    	szLeft = szCurLine
    	j = 0
    	while(j < tmpLen)
    	{
    		if(szCurLine[j] != " " && szCurLine[j] != "\t" && szCurLine[j] != "/")
    		{	 
				szLeft = strmid(szCurLine,0,j)
				szLeft = cat(szLeft, "//")
				szRight = strmid(szCurLine,j,strlen(szCurLine))
				szLeft = cat(szLeft, szRight)
    			DelBufLine(hbuf, curLineNum)
    			InsBufLine(hbuf, curLineNum, "@szLeft@")
    			break
    		}
    		else
    		{
				if(szCurLine[j] == "/" && szCurLine[j + 1] == "/")
				{
					szLeft = strmid(szCurLine,0,j)
					szRight = strmid(szCurLine, j + 2, strlen(szCurLine))
					szLeft = cat(szLeft, szRight)
    				DelBufLine(hbuf, curLineNum)
    				InsBufLine(hbuf, curLineNum, "@szLeft@")
    				break
				}else{
					j = j + 1
					continue
				}
    		}
    		j = j + 1
    	}
    	curLineNum = curLineNum + 1
    }
}


/*!
 * \fn     O_BlockComment
 * \brief  块注释
 *          
 * 
 * \retval 
 */

/************************************************
 change history: 
    1.date  : 2018年3月24日
      author: Ouqichen
      change: create function

************************************************/
macro O_BlockComment()
{
	hwnd = GetCurrentWnd()
    if (hwnd == 0)
        stop 
    sel = GetWndSel(hwnd)
	hbuf = GetWndBuf(hwnd)
	
    FirstLineNum = sel.lnFirst 
	LastLineNum = sel.lnLast

	//由于先处理前面的 /* 会导致行数增加，导致后买的*/处理起来行数不确定，处理困难，因此先处理 */ */的增加删除不会引起/*行号变动
	szLine = GetBufLine(hbuf, LastLineNum + 1)
	if(strstr(szLine,"*/") == 0xffffffff)
	{
		InsBufLine(hbuf, LastLineNum + 1, "*/")
	}
	else
	{
		DelBufLine(hbuf, LastLineNum + 1)
	}

	//处理*/
	szLine = GetBufLine(hbuf, FirstLineNum - 1)
	if(strstr(szLine,"/*") == 0xffffffff)
	{
		InsBufLine(hbuf, FirstLineNum , "/*")
	}
	else
	{
		DelBufLine(hbuf, FirstLineNum - 1)
	}
}

// Ask user for ifdef condition and wrap it around current
// selection.
macro O_InsertIfdef()
{
	sz = Ask("请输入定义项")
	if (sz != "")
		IfdefSz(sz);
}

// Wrap ifdef <sz> .. endif around the current selection
macro IfdefSz(sz)
{
	hwnd = GetCurrentWnd()
	lnFirst = GetWndSelLnFirst(hwnd)
	lnLast = GetWndSelLnLast(hwnd)
	 
	hbuf = GetCurrentBuf()
	InsBufLine(hbuf, lnFirst, "#ifdef @sz@")
	InsBufLine(hbuf, lnLast+2, "#endif /* @sz@ */")
}

/*!
 * \fn     HeadIfdefSz
 * \brief  头宏注释
 *          
 * \param  [in] sz   #
 * 
 * \retval 
 */

/************************************************
 change history: 
    1.date  : 2018年3月24日
      author: Ouqichen
      change: create function

************************************************/
macro HeadIfdefSz(sz)
{
    hwnd = GetCurrentWnd()
    lnFirst = GetWndSelLnFirst(hwnd)
    hbuf = GetCurrentBuf()
    InsBufLine(hbuf, lnFirst, "")
    InsBufLine(hbuf, lnFirst, "#define @sz@")
    InsBufLine(hbuf, lnFirst, "#ifndef @sz@")
    iTotalLn = GetBufLineCount (hbuf)                
    InsBufLine(hbuf, iTotalLn, "#endif /* @sz@ */")
    InsBufLine(hbuf, iTotalLn, "")
}

/*!
 * \fn     GetFileNameNoExt
 * \brief  得到无后缀的文件名
 *          
 * \param  [in] sz   #
 * 
 * \retval 
 */

/************************************************
 change history: 
    1.date  : 2018年3月24日
      author: Ouqichen
      change: create function

************************************************/
macro GetFileNameNoExt(sz)
{
    i = 1
    szName = sz
    iLen = strlen(sz)
    j = iLen 
    if(iLen == 0)
      return ""
    while( i <= iLen)
    {
      if(sz[iLen-i] == ".")
      {
         j = iLen-i 
      }
      if( sz[iLen-i] == "\\" )
      {
         szName = strmid(sz,iLen-i+1,j)
         return szName
      }
      i = i + 1
    }
    szName = strmid(sz,0,j)
    return szName
}

/*!
 * \fn     O_CreateHeadFile
 * \brief  生成头文件
 *          
 * 
 * \retval 
 */

/************************************************
 change history: 
    1.date  : 2018年3月24日
      author: Ouqichen
      change: create function

************************************************/
macro O_CreateHeadFile()
{
	hwnd = GetCurrentWnd()
    if (hwnd == 0)
        stop 
    hbuf = GetWndBuf(hwnd)
	szFileName = GetFileNameNoExt(GetBufName(hbuf))
	if(strlen(szFileName) == 0)
    { 
    	msg("源文件无文件名，无法生成对应的头文件")
    	stop
    }
	szPreH = toupper (szFileName)
    szHeadFile = cat(szFileName,".h")
    szPreH = cat(szPreH,"_H")
    hOutbuf = NewBuf(szHeadFile) // create output buffer
    if (hOutbuf == 0)
        stop
    //搜索符号表取得函数名
    SetCurrentBuf(hOutbuf)
	HeadIfdefSz(szPreH)
	
	szMyName = getreg(MYNAME)
    if(strlen( szMyName ) == 0)
    {
        szMyName = Ask("请输入你的名字：")
        setreg(MYNAME, szMyName)
    }
    szOwner = getreg(OWNER)
   if(strlen(szOwner) == 0)
	{
		szOwner = ASK("请输入版权所有者")
		SetReg ("OWNER", szOwner)		
	}
    InsertFileHead(hOutbuf,0, szMyName,"@szFileName@.c 的头文件",szOwner)
}

/*!
 * \fn     O_CreatePrintFile
 * \brief  生成打印头文件
 *          
 * 
 * \retval 
 */

/************************************************
 change history: 
    1.date  : 2018年3月24日
      author: Ouqichen
      change: create function

************************************************/
macro O_CreatePrintFile()
{
	hOutbuf = NewBuf("print.h") // create output buffer
    if (hOutbuf == 0)
        stop
    //搜索符号表取得函数名
    SetCurrentBuf(hOutbuf)
    ln = 28
    HeadIfdefSz("PRINT_H")
    
    szMyName = getreg(MYNAME)
    if(strlen( szMyName ) == 0)
    {
        szMyName = Ask("请输入你的名字：")
        setreg(MYNAME, szMyName)
    }
    szOwner = getreg(OWNER)
   if(strlen(szOwner) == 0)
	{
		szOwner = ASK("请输入版权所有者")
		SetReg ("OWNER", szOwner)		
	}
    InsertFileHead(hOutbuf,0, szMyName,"打印宏",szOwner)
    
    InsBufLine(hOutbuf, ln, "#define DEBUG")
    InsBufLine(hOutbuf, ln + 1, "#define ERROR")
    InsBufLine(hOutbuf, ln + 2, "#define WARNING")
    InsBufLine(hOutbuf, ln + 3, "#define INFO")
    InsBufLine(hOutbuf, ln + 4, "#define MSG")
    
    InsBufLine(hOutbuf, ln + 5, "")
    
    InsBufLine(hOutbuf, ln + 6, "#ifdef DEBUG")
    InsBufLine(hOutbuf, ln + 7, "#define DEBUG_MSG(format, args…) printf(\"DEBUG(%s;%d;%s): \"format\"\\n\", __FILE__, __LINE__, __FUNCTION__, ##args)")
    InsBufLine(hOutbuf, ln + 8, "#else")
    InsBufLine(hOutbuf, ln + 9, "#define DEBUG_MSG(format, args…)")
    InsBufLine(hOutbuf, ln + 10, "#endif")

    InsBufLine(hOutbuf, ln + 11, "")
    
    InsBufLine(hOutbuf, ln + 12, "#ifdef ERROR")
    InsBufLine(hOutbuf, ln + 13, "#define ERROR_MSG(format, args…) printf(\"ERROR: \"format\"\\n\", ##args)")
    InsBufLine(hOutbuf, ln + 14, "#else")
    InsBufLine(hOutbuf, ln + 15, "#define ERROR_MSG(format, args…)")
    InsBufLine(hOutbuf, ln + 16, "#endif")

    InsBufLine(hOutbuf, ln + 17, "")
    
    InsBufLine(hOutbuf, ln + 18, "#ifdef WARNING")
    InsBufLine(hOutbuf, ln + 19, "#define WARNING_MSG(format, args…) printf(\"WARNING: \"format\"\\n\", ##args)")
    InsBufLine(hOutbuf, ln + 20, "#else")
    InsBufLine(hOutbuf, ln + 21, "#define WARNING_MSG(format, args…)")
    InsBufLine(hOutbuf, ln + 22, "#endif")

    InsBufLine(hOutbuf, ln + 23, "")
    
    InsBufLine(hOutbuf, ln + 24, "#ifdef INFO")
    InsBufLine(hOutbuf, ln + 25, "#define INFO_MSG(format, args…) printf(\"INFO: \"format\"\\n\", ##args)")
    InsBufLine(hOutbuf, ln + 26, "#else")
    InsBufLine(hOutbuf, ln + 27, "#define INFO_MSG(format, args…)")
    InsBufLine(hOutbuf, ln + 28, "#endif")

    InsBufLine(hOutbuf, ln + 29, "")

    InsBufLine(hOutbuf, ln + 30, "#ifdef MSG")
    InsBufLine(hOutbuf, ln + 31, "#define MSG(args…) printf(args)")
    InsBufLine(hOutbuf, ln + 32, "#else")
    InsBufLine(hOutbuf, ln + 33, "#define MSG(args…)")
    InsBufLine(hOutbuf, ln + 34, "#endif")
    
    InsBufLine(hOutbuf, ln + 35, "")
    InsBufLine(hOutbuf, ln + 36, "")
    InsBufLine(hOutbuf, ln + 37, "")
}

